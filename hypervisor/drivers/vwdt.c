/*
 * Copyright (C) 2018 Min Le (lemin9538@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <minos/minos.h>
#include <minos/vdev.h>
#include <asm/io.h>
#include <minos/irq.h>
#include <minos/timer.h>
#include <minos/gvm.h>

#define SP805_WDT_LOAD		0x00
#define SP805_WDT_VALUE		0x04
#define SP805_WDT_CTL		0x08
#define SP805_WDT_INTCLR	0x0c
#define SP805_WDT_RIS		0x10
#define SP805_WDT_MIS		0x14
#define SP805_WDT_LOCK		0xc00
#define SP805_WDT_ITCR		0xf00
#define SP805_WDT_ITOP		0xf04
#define SP805_WDT_PRID0		0xfe0
#define SP805_WDT_PRID1		0xfe4
#define SP805_WDT_PRID2		0xfe8
#define SP805_WDT_PRID3		0xfec
#define SP805_WDT_PCID0		0xff0
#define SP805_WDT_PCID1		0xff4
#define SP805_WDT_PCID2		0xff8
#define SP805_WDT_PCID3		0xffc

#define LOAD_MIN		0x00000001
#define LOAD_MAX		0xffffffff
#define INT_ENABLE		(1 << 0)
#define RESET_ENABLE		(1 << 1)
#define INT_MASK		(1 << 0)
#define WDT_LOCK		0x00000001
#define WDT_UNLOCK		0x1acce551

struct vwdt_dev {
	uint8_t int_enable;
	uint8_t reset_enable;
	uint8_t int_trigger;
	uint8_t access_lock;
	uint32_t load_value;
	unsigned long timeout;
	struct timer_list wdt_timer;
	struct vdev vdev;
};

#define vdev_to_vwdt(vdev) \
	(struct vwdt_dev *)container_of(vdev, struct vwdt_dev, vdev);

static int vwdt_mmio_read(struct vdev *vdev, gp_regs *regs,
		unsigned long address, unsigned long *value);
{
	return 0;
}

static void vwdt_timer_expire(unsigned long data)
{
	struct vwdt_dev *wdt = (struct vwdt_dev *)data;

	/* if the timeout int has already triggered reset
	 * the system if the reset is enabled */
	if (wdt->int_trigger && wdt->reset_enable) {
		if (vm_is_native(wdt->vdev.vm))
			panic("native vm watchdog timeout\n");
		else {
			trap_vcpu_nonblock();
			sched();
		}
	} else {
		if (wdt->int_enable) {
			send_virq_to_vm(wdt->vdev.vm, SP805_IRQ);
			wdt->int_trigger = 1;
			mod_timer(&wdt->timer, NOW() + wdt->timeout);
		}
	}
}

static void inline vwdt_config_ctrl(struct vwdt_dev *wdt,
		int int_enable, int reset_enable)
{
	if (wdt->int_enable != int_enable) {
		wdt->int_enable = int_enable;
		if (int_enable == 0)
			wdt->int_trigger = 0;

		if (int_eanble)
			mod_timer(&wdt->wdt_timer, NOW() + wdt->timeout);
		else
			del_timer(&wdt->wdt_timer);
	}

	if (wdt->reset_enable != reset_enalbe)
		wdt->reset_enable = reset_enable;
}

static void inline vwdt_config_intclr(struct vwdt_dev *wdt)
{
	/* reload the value and restart the timer */
	wdt->int_trigger = 0;
	mod_timer(&wdt->wdt_timer, NOW() + wdt->timeout);
}

static int vwdt_mmio_write(struct vdev *vdev, gp_regs *regs,
		unsigned long address, unsigned long *value)
{
	uint32_t v = (uint32_t)(*value);
	struct vwdt_dev *wdt = vdev_to_vwdt(vdev);
	unsigned long offset = address - VWDT_IOMEM_BASE;

	if ((offset != SP805_WDT_LOCK) && wdt->access_lock) {
		pr_error("register is locked of the wdt\n");
		return -EPERM;
	}

	switch (offset) {
	case SP805_WDT_LOAD:
		wdt->load_value = v;
		break;
	case SP805_WDT_CTL:
		vwdt_config_ctrl(wdt, !!(v & INT_ENABLE),
				!!(v & RESET_ENABLE));
		break;
	case SP805_WDT_INTCLR:
		vwdt_config_intclr(wdt);
		break;
	case SP805_WDT_LOCK:
		if (v == WDT_UNLOCK)
			wdt->access_lock = 0;
		else if (v == WDT_LOCK)
			wdt->access_lock = 1;
		else
			pr_warn("unsupport value of wdt_lock\n");
		break;
	default:
		break;
	}

	return 0;
}

static void vwdt_reset(struct vdev *vdev)
{
	pr_info("vwdt reset\n");
}

static void vwdt_deinit(struct vdev *vdev)
{
	struct vwdt_dev *dev = vdev_to_vwdt(vdev);

	vdev_release(&dev->vdev);
	free(dev);
}

static int vwdt_create_vm(void *item, void *arg)
{
	struct vm *vm = (struct vm *)item;
	struct vwdt_dev *dev;
	struct vcpu *vcpu = get_vcpu_in_vm(vm, 0);

	if (vm_is_hvm(vm))
		return 0;

	pr_info("create virtual watchdog for vm-%d\n", vm->vmid);
	
	dev = zalloc(sizeof(struct vwdt_dev));
	if (!dev)
		return -ENOMEM;

	host_vdev_init(vm, &dev->vdev, SP805_IOMEM_BASE,
			SP805_IOMEM_SIZE);
	vdev_set_name(&dev->vdev, "vwdt");

	dev->vdev.read = vwdt_mmio_read;
	dev->vdev.write = vwdt_mmio_write;
	dev->vdev.deinit = vwdt_deinit;
	dev->vdev.reset = vwdt_reset;

	init_timer_on_cpu(&dev->wdt_timer, vcpu->affinity);
	dev->wdt_timer.function = vwdt_timer_expire;
	dev->wdt_timer.data = (unsigned long)dev;

	return 0;
}

int vwdt_init(void)
{
	return register_hook(vwdt_create_vm,
			MINOS_HOOK_TYPE_CREATE_VM_VDEV);
}

device_initcall(vwdt_init);
