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

#include <minos/sched.h>
#include <minos/minos.h>
#include <minos/percpu.h>
#include <minos/pm.h>
#include <minos/irq.h>
#include <minos/list.h>
#include <minos/timer.h>
#include <minos/time.h>
#include <minos/virt.h>
#include <minos/virq.h>

extern void sched_tick_disable(void);
extern void sched_tick_enable(unsigned long exp);

static struct pcpu pcpus[NR_CPUS];

DEFINE_PER_CPU(struct pcpu *, pcpu);
DEFINE_PER_CPU(struct vcpu *, percpu_current_vcpu);
DEFINE_PER_CPU(struct vcpu *, percpu_next_vcpu);

DEFINE_PER_CPU(int, need_resched);
DEFINE_PER_CPU(atomic_t, preempt);

static int e_base;
static int f_base;
static int e_base_current;
static int f_base_current;
static int e_nr;
static int f_nr;
DEFINE_SPIN_LOCK(affinity_lock);

void get_vcpu_affinity(uint8_t *aff, int nr)
{
	int i, e, f, j = 0;

	e = MIN(nr, e_nr);
	f = nr - e;

	spin_lock(&affinity_lock);

	for (i = 0; i < e; i++) {
		aff[j] = e_base_current;
		e_base_current++;
		if (e_base_current == NR_CPUS)
			e_base_current = e_base;
		j++;
	}

	for (i = 0; i < f; i++) {
		if ( (f_base_current == 0) && ((f < f_nr) || (i == 0)) ) {
			f_base_current++;
			if (f_base_current == e_base)
				f_base_current = f_base;
		}

		aff[j] = f_base_current;
		j++;
		f_base_current++;
		if (f_base_current == e_base)
			f_base_current = f_base;
	}

	spin_unlock(&affinity_lock);
}

static int vcpu_affinity_init(void)
{
	struct vm *vm0 = get_vm_by_id(0);

	if (!vm0)
		panic("vm0 is not created\n");

	e_base_current = e_base = vm0->vcpu_nr;
	e_nr =  NR_CPUS - vm0->vcpu_nr;
	f_base_current = f_base = 0;
	f_nr = vm0->vcpu_nr;

	return 0;
}
device_initcall(vcpu_affinity_init);

void pcpu_resched(int pcpu_id)
{
	send_sgi(CONFIG_MINOS_RESCHED_IRQ, pcpu_id);
}

void switch_to_vcpu(struct vcpu *current, struct vcpu *next)
{
	struct pcpu *pcpu = get_cpu_var(pcpu);

	if (current != next) {
		if (!current->is_idle)
			save_vcpu_state(current);

		if (!next->is_idle)
			restore_vcpu_state(next);

		pcpu->sched_class->sched(pcpu, current, next);

		/*
		 * if the current vcpu's state is running, indicate
		 * the switch is caused by timer tick, change the
		 * state here, otherwise is caused by the state
		 * change by other reason
		 */
		if (current->state == VCPU_STAT_RUNNING)
			current->state = VCPU_STAT_READY;

		next->state = VCPU_STAT_RUNNING;
	}

	if (!next->is_idle)
		enter_to_guest(next, NULL);
}

void kick_vcpu(struct vcpu *vcpu)
{
	struct vcpu *current = current_vcpu;

	/*
	 * if the vcpu is in suspend or idle state
	 * when recevie new interrput or other things
	 * which cause vcpu is waked up, need to set
	 * the vcpu's to ready again
	 */
	if ((vcpu == current) || vcpu->is_idle)
		return;

	if (vcpu->state == VCPU_STAT_SUSPEND) {
		if (vcpu->affinity != current->affinity) {
			vcpu->resched = 1;
			pcpu_resched(vcpu->affinity);
		} else
			set_vcpu_state(vcpu, VCPU_STAT_READY);
	}
}

void sched_new(void)
{
	struct pcpu *pcpu = get_cpu_var(pcpu);

	next_vcpu = pcpu->sched_class->pick_vcpu(pcpu);
}

void sched(void)
{
	unsigned long flags;
	struct vcpu *vcpu, *current = current_vcpu;
	struct pcpu *pcpu;

	pcpu = get_cpu_var(pcpu);

	local_irq_save(flags);
	vcpu = pcpu->sched_class->pick_vcpu(pcpu);
	local_irq_restore(flags);

	if (vcpu != current) {
		local_irq_save(flags);
		switch_to_vcpu(current, vcpu);
		next_vcpu = vcpu;
		dsb();
		arch_switch_vcpu_sw();
		local_irq_restore(flags);
	}
}

void pcpus_init(void)
{
	int i;
	struct pcpu *pcpu;

	for (i = 0; i < NR_CPUS; i++) {
		pcpu = &pcpus[i];
		memset(pcpu, 0, sizeof(struct pcpu));
		pcpu->state = PCPU_STATE_OFFLINE;
		init_list(&pcpu->vcpu_list);
		pcpu->pcpu_id = i;
		get_per_cpu(pcpu, i) = pcpu;
		spin_lock_init(&pcpu->lock);
	}
}

void set_vcpu_state(struct vcpu *vcpu, int state)
{
	int a, b;
	unsigned long flags;
	int old_state = vcpu->state;
	struct pcpu *pcpu = get_per_cpu(pcpu, vcpu->affinity);

	if ((old_state == state) || (vcpu->is_idle))
		return;

	/*
	 * set the vcpu to the new state, and update the pcpu
	 * information about the running vcpus, now only support
	 * ready, suspend and idle state
	 */
	pcpu->sched_class->set_vcpu_state(pcpu, vcpu, state);

	local_irq_save(flags);

	state = vcpu->state;
	a = (old_state == VCPU_STAT_READY) ||
		(old_state == VCPU_STAT_RUNNING);
	b = (state == VCPU_STAT_SUSPEND) || (state == VCPU_STAT_IDLE);

	/*
	 * if the vcpu's state from ready/running to suspended
	 * or idle decreae the pcpu's running vcpus
	 */
	if (a && b) {
		pcpu->nr_running_vcpus--;
		if (pcpu->nr_running_vcpus == 1) {
			pr_info("disable sched_timer\n");
			sched_tick_disable();
		}
	} else if ((!a) && (!b)) {
		pcpu->nr_running_vcpus++;
		if (pcpu->nr_running_vcpus == 2) {
			pr_info("enable sched_timer\n");
			sched_tick_enable(pcpu->sched_class->sched_interval);
		}
	}

	local_irq_restore(flags);
}

int sched_reset_vcpu(struct vcpu *vcpu)
{
	struct pcpu *pcpu = get_per_cpu(pcpu, vcpu->affinity);

	if (pcpu->sched_class->reset_vcpu_data)
		pcpu->sched_class->reset_vcpu_data(pcpu, vcpu);

	return 0;
}

int pcpu_remove_vcpu(int cpu, struct vcpu *vcpu)
{
	int ret;
	struct pcpu *pcpu;

	if (cpu >= NR_CPUS) {
		pr_error("No such physical cpu:%d\n", cpu);
		return -EINVAL;
	}

	if (vcpu->affinity != cpu) {
		pr_error("vcpu don not belong to this pcpu\n");
		return -EINVAL;
	}

	pcpu = get_per_cpu(pcpu, cpu);

	ret = pcpu->sched_class->remove_vcpu(pcpu, vcpu);
	if (ret) {
		pr_error("remove the vcpu from pcpu failed\n");
		return ret;
	}

	/* deinit the vcpu's sched private data */
	pcpu->sched_class->deinit_vcpu_data(pcpu, vcpu);

	spin_lock(&pcpu->lock);
	list_del(&vcpu->list);
	pcpu->nr_vcpus--;
	spin_unlock(&pcpu->lock);

	return 0;
}

int pcpu_add_vcpu(int cpu, struct vcpu *vcpu)
{
	int ret;
	struct pcpu *pcpu;

	if (cpu >= NR_CPUS) {
		pr_error("No such physical cpu:%d\n", cpu);
		return -EINVAL;
	}

	pcpu = get_per_cpu(pcpu, cpu);

	/* init the vcpu's sched private data */
	pcpu->sched_class->init_vcpu_data(pcpu, vcpu);

	ret = pcpu->sched_class->add_vcpu(pcpu, vcpu);
	if (ret) {
		pr_error("add vcpu to pcpu failed\n");
		spin_lock(&pcpu->lock);
		list_del(&vcpu->list);
		spin_unlock(&pcpu->lock);

		pcpu->sched_class->deinit_vcpu_data(pcpu, vcpu);
		return ret;
	}

	spin_lock(&pcpu->lock);
	list_add_tail(&pcpu->vcpu_list, &vcpu->list);
	pcpu->nr_vcpus++;
	spin_unlock(&pcpu->lock);

	return 0;
}

static int resched_handler(uint32_t irq, void *data)
{
	struct pcpu *pcpu = get_cpu_var(pcpu);
	struct vcpu *vcpu;
	int state;

	list_for_each_entry(vcpu, &pcpu->vcpu_list, list) {
		if (vcpu->resched) {
			state = vcpu->state;
			if ((state != VCPU_STAT_READY) && (state != VCPU_STAT_RUNNING))
				set_vcpu_state(vcpu, VCPU_STAT_READY);

			/* ensure to clear the resched flag */
			vcpu->resched = 0;
		}
	}

	if (pcpu->sched_class->flags & SCHED_FLAGS_PREEMPT)
		need_resched = 1;

	return 0;
}

unsigned long sched_tick_handler(unsigned long data)
{
	unsigned long ticks;
	struct pcpu *pcpu = get_cpu_var(pcpu);

	ticks = pcpu->sched_class->tick_handler(pcpu);
	if (pcpu->nr_running_vcpus <= 1)
		return 0;

	return ticks;
}

int sched_init(void)
{
	int i;
	struct pcpu *pcpu;

	for (i = 0; i < NR_CPUS; i++) {
		atomic_set(&get_per_cpu(preempt, i), 0);
		pcpu = get_per_cpu(pcpu, i);
		pcpu->sched_class = get_sched_class("fifo");
		pcpu->sched_class->init_pcpu_data(pcpu);
	}

	return 0;
}

int local_sched_init(void)
{
	struct pcpu *pcpu = get_cpu_var(pcpu);

	pcpu->state = PCPU_STATE_RUNNING;

	return request_irq(CONFIG_MINOS_RESCHED_IRQ, resched_handler,
			0, "resched handler", NULL);
}
