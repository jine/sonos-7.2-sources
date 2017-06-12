/*
 * Bulverde Performance profiler and Idle profiler Routines
 *
 * Copyright (c) 2003 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

/*
#undef DEBUG
#define DEBUG
*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <asm/proc-fns.h>
#include <asm/mach/time.h>
#include <asm/arch/ipmc.h>
#include <asm/arch/ipm-profiler.h>

#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif

#define MAX_OSCR0		0xFFFFFFFF
/* 1ms in ticks (3.25x10^6tick/sec * 1/1000sec/msec)	*/
#define OSCR0_TICKS_1MS		3250

#define MAX_LENGTH		20

static DECLARE_MUTEX_LOCKED(ipm_sem);

extern int deepidle_ticks;
extern int pxa_dyn_enable;

extern int pxa3xx_fv_enter_low_op(void);
extern int pxa3xx_fv_exit_low_op(void);
extern int is_low_op(void);

extern int ipm_event_notify(int type, int kind, void *info,
		unsigned int info_len);
extern int (*pipm_start_pmu)(struct ipm_profiler_arg *arg);

static int idle_flaw = 0;	/* silicon issue on IDLE */
static int ipm_thread_exit = 0;
static int pmu_id = 0;
static unsigned long deep_ticks = 1;
struct completion ipm_thread_over;
static struct ipm_profiler_arg cur_arg;

static void (*orig_idle)(void) = NULL;

/* Idle Profiler specific variables	*/
static unsigned int aggregate_idle_time	= 0;

#define DEBUG_IDLE_COUNT
#ifdef DEBUG_IDLE_COUNT
static unsigned long idle_cnt = 0;
static unsigned long idle_ticks = 0;
#endif


#ifdef TRACE
static void print_pmu_reg(struct pmu_results *presult)
{
	pr_debug("ccnf_of: 0x%08x  ", presult->ccnt_of);
	pr_debug("ccnt: 0x%08x\n", presult->ccnt);

	pr_debug("pmn0_of: 0x%08x  ", presult->pmn0_of);
	pr_debug("pmn0: 0x%08x\n", presult->pmn0);

	pr_debug("pmn1_of: 0x%08x  ", presult->pmn1_of);
	pr_debug("pmn1: 0x%08x\n", presult->pmn1);

	pr_debug("pmn2_of: 0x%08x  ", presult->pmn2_of);
	pr_debug("pmn2: 0x%08x\n", presult->pmn2);

	pr_debug("pmn3_of: 0x%08x  ", presult->pmn3_of);
	pr_debug("pmn3: 0x%08x\n", presult->pmn3);
}
#else
#define print_pmu_reg(args)	do {} while (0)
#endif


int ipm_start_profiler(struct ipm_profiler_arg *arg)
{
	if (arg == NULL) {
		up(&ipm_sem);
		return -EINVAL;
	}
	memset(&cur_arg, 0, sizeof(struct ipm_profiler_arg));
	cur_arg.flags = arg->flags;
	if (arg->window_size > 0)
		cur_arg.window_size = arg->window_size;
	else
		cur_arg.window_size = DEFAULTWINDOWSIZE;
	if (arg->flags & IPM_PMU_PROFILER) {
		cur_arg.pmn0 = arg->pmn0;
		cur_arg.pmn1 = arg->pmn1;
		cur_arg.pmn2 = arg->pmn2;
		cur_arg.pmn3 = arg->pmn3;
	}
	up(&ipm_sem);
	return 0;
}

void ipm_deepidle(unsigned long ticks)
{
	deep_ticks = ticks;
}

/* check whether current operating point is idle flaw operating point */
static int is_idle_flaw_op(void)
{
	if (idle_flaw) {
		if (ACSR & ACCR_D0CS_MASK)
			return 1;
		if (((ACSR & ACCR_XL_MASK) == 8) && ((ACSR & ACCR_XN_MASK) ==
			(1 << ACCR_XN_OFFSET))) {
			/* 104MHz */
			return 1;
		}
	}
	return 0;
}

static void pxa3xx_cpu_idle(void)
{
	unsigned int icpr, icpr2, icmr, icmr2, iccr;
	if (is_idle_flaw_op()) {
		/* Loop and query interrupt.
		 * At here, only IRQ is awared. FIQ is ignored.
		 */
		iccr = ICCR;
		while (1) {
			__asm__ __volatile__ ("\n\
					mrc     p6, 0, %0, c4, c0, 0    @ Read out ICPR\n\
					mrc     p6, 0, %1, c10, c0, 0   @ Read out ICPR2\n\
					mrc     p6, 0, %2, c1, c0, 0    @ Read out ICMR\n\
					mrc     p6, 0, %3, c7, c0, 0    @ Read out ICMR2\n"
					:"=&r"(icpr),"=&r"(icpr2),"=&r"(icmr),"=&r"(icmr2)
					:
					:"memory","cc"
					);
			if (iccr & 0x1) {
				if (((icpr & icmr) != 0) || ((icpr2 & icmr2) != 0)) break;
			} else {
				if ((icpr != 0) || (icpr2 != 0)) break;
			}
		};
	} else
		cpu_do_idle();
}

static int beyond_d0cs_tick(void)
{
	int ret = 0;
	if (pxa_dyn_enable) {
		if (deep_ticks >= deepidle_ticks)
			ret = 1;
	} else {
		/* always enable deep idle */
		ret = 1;
	}
	return ret;
}

/*
 *	ipm idle profiler idle thread.
 */
void ipm_ip_idle(void)
{
	unsigned long curr_idle_start_time, curr_idle_stop_time,
			curr_time_idle;

	local_irq_disable();

	curr_idle_start_time = OSCR;

	/* value of current in this context?	*/
	if(!need_resched() && !hlt_counter) {
		timer_dyn_reprogram();
		/* Set low op before entering idle. Exit after leaving idle. */
		if (enable_deepidle) {
			if (beyond_d0cs_tick() && !is_low_op()) {
				pxa3xx_fv_enter_low_op();
				pxa3xx_cpu_idle();
				pxa3xx_fv_exit_low_op();
			} else
				pxa3xx_cpu_idle();
			deep_ticks = 1;
		} else
			pxa3xx_cpu_idle();
	}

	curr_idle_stop_time = OSCR;
	if (curr_idle_stop_time > curr_idle_start_time)
		curr_time_idle = (curr_idle_stop_time - curr_idle_start_time);
	else
		curr_time_idle = MAX_OSCR0 - curr_idle_start_time
			+ curr_idle_stop_time;
	aggregate_idle_time += curr_time_idle;
#ifdef DEBUG_IDLE_COUNT
	idle_cnt++;
	idle_ticks += curr_time_idle;
#endif
	local_irq_enable();
}

/*
 * WindowSize is defined in idle-prof.c so two profilers are using
 * the same window size.
 */
static int ipm_thread(void *data)
{
	struct task_struct *tsk = current;
	int valid;
	unsigned long window_size_jiffies;
	unsigned long timeout;

	/* init PMU */
	pmu_id = pmu_claim();

	daemonize("ipmd");
	strcpy(tsk->comm, "ipmd");
	/*
	 * We run it as a real-time thread.
	 */
	tsk->policy = SCHED_FIFO;
	tsk->rt_priority = 1;
	tsk->flags |= PF_NOFREEZE;

	/* only want to receive SIGKILL */
	allow_signal(SIGKILL);
	valid = 0;

	memset(&cur_arg, 0, sizeof(struct ipm_profiler_arg));
	cur_arg.window_size = DEFAULTWINDOWSIZE;

	while(1)  {
		unsigned long	interval;
		int event_kind = 0;
		struct ipm_profiler_result result;

		/* Get blocked.*/
		if (down_interruptible(&ipm_sem)) {
			if (pmu_id)
				pmu_release(pmu_id);
			complete_and_exit(&ipm_thread_over, -EINTR);
			return -EINTR;
		}
		if (ipm_thread_exit)
			break;

		aggregate_idle_time = 0;

		pr_debug("ws=%d pmn0=%d pmn1=%d pmn2=%d\n",
				cur_arg.window_size, cur_arg.pmn0,
				cur_arg.pmn1, cur_arg.pmn2);
		if (cur_arg.flags & IPM_PMU_PROFILER) {
			/* monitor PMU events */
			if (pmu_start(cur_arg.pmn0, cur_arg.pmn1, cur_arg.pmn2,
					cur_arg.pmn3)) {
				pr_debug("pmu_start: failed!\n");
		        	up(&ipm_sem);
				continue;
			}
			else {
				pr_debug("pmu_start: success!\n");
			}
			event_kind |= IPM_PMU_PROFILER;
		}

		interval = jiffies;
		window_size_jiffies = cur_arg.window_size * HZ / 1000;
		timeout = jiffies + window_size_jiffies;

		while (time_before(jiffies, timeout)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(window_size_jiffies);
			/*
			 * Make the current task sleep until @timeout jiffies
			 * have elapsed.  (jiffie = 10ms)
			 * The routine will return immediately the current
			 * task state has been set, see set_current_state().
			 */

			if(signal_pending(tsk))
				break;
		}

		if (cur_arg.flags & IPM_PMU_PROFILER) {
			if (pmu_stop(&(result.pmu))) {
				pr_debug("pmu_stop: failed!\n");
				up(&ipm_sem);
				continue;
			}
			else {
				pr_debug("pmu_stop: success!\n");
			}
		}

		/* Window has ended; calculate CPU% */
		interval = jiffies - interval;
		result.window_size = interval * 1000 / HZ;
		if (cur_arg.flags & IPM_IDLE_PROFILER) {
			if ((aggregate_idle_time < 0) ||
					(result.window_size <=0)) {
				pr_debug("ipm profiler get parameter with \
						zero.\n");
				result.busy_ratio = 0;
			} else if ((aggregate_idle_time == 0)
				&& (result.window_size > 0)) {
				result.busy_ratio = 100;
			} else {
				result.busy_ratio =  100 * (result.window_size
						- aggregate_idle_time
						/ OSCR0_TICKS_1MS)
						/ result.window_size;
				pr_debug("result.ws=%d agg_idle=%d \
						result.busy=%d\n",
						result.window_size,
						aggregate_idle_time,
						result.busy_ratio);
			}
			event_kind |= IPM_IDLE_PROFILER;
		}

		ipm_event_notify(IPM_EVENT_PROFILER, event_kind, &result,
				sizeof(struct ipm_profiler_result));

	}

	if( pmu_id )
		pmu_release( pmu_id);

	complete_and_exit(&ipm_thread_over, 0);

	return 0;
}

static struct proc_dir_entry *entry_dir = NULL;
#ifdef DEBUG_IDLE_COUNT
static struct proc_dir_entry *entry_idle = NULL;

static int proc_read_idle(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
	int len;

	len = sprintf(page, "ipm idle count:%ld ticks:%ld\n", idle_cnt,
			idle_ticks);
	return len;
}
#endif

static int ipm_proc_init(void)
{
	entry_dir = proc_mkdir("driver/ipm", NULL);
	if (entry_dir == NULL) {
		return -ENOMEM;
	}

#ifdef DEBUG_IDLE_COUNT
	entry_idle = create_proc_read_entry("idle", 0444, entry_dir,
						proc_read_idle, NULL);
	if (entry_idle == NULL) {
		remove_proc_entry("driver/ipm", NULL);
		return -ENOMEM;
	}
#endif

	return 0;
}

static void ipm_proc_cleanup(void)
{
#ifdef DEBUG_IDLE_COUNT
	remove_proc_entry("idle", entry_dir);
#endif
	remove_proc_entry("driver/ipm", NULL);
}

#ifdef CONFIG_DPM
static int ipm_idle_load(void)
{
	if (down_trylock(&_dpm_lock)) {
		/* DPM is operating on _dpm_lock. We should make sure that DPM
		 * have terminated
		 */
		return -EBUSY;
	}
	orig_idle = pm_idle;
	pm_idle = ipm_ip_idle;
	up(&_dpm_lock);
	return 0;
}

static void ipm_idle_clean(void)
{
	dynamicpower_terminate();
	down(&_dpm_lock);
	pm_idle = orig_idle;
	up(&_dpm_lock);
}
#else
static int ipm_idle_load(void)
{
	orig_idle = pm_idle;
	pm_idle = ipm_ip_idle;
	return 0;
}

static void ipm_idle_clean(void)
{
	pm_idle = orig_idle;
}
#endif

/*
 * The callback for timer used by idle (and eventually perf) profilers
 */

/*
 * 	Initialize the PMU profiler
 *	Start the PMU_Profiler Thread
 */
static int __init ipm_init(void)
{
	unsigned int ret = 0;
	unsigned int cpuid;

	/* check idle flaw */
	cpuid = read_cpuid(0) & 0xFFFF;
	if ((cpuid >= 0x6880) && (cpuid <= 0x6881)) {
		/* PXA300 A0/A1 */
		idle_flaw = 1;
	} else if ((cpuid >= 0x6890) && (cpuid <= 0x6892)) {
		/* PXA310 A0/A1/A2 */
		idle_flaw = 1;
	} else
		idle_flaw = 0;
	if (idle_flaw) printk("This system is using the idle flaw workaround\n");
	if (ipm_idle_load()) {
		up(&ipm_sem);
		return -EFAULT;
	}

	/* Create file in procfs */
	if (ipm_proc_init()) {
		ipm_idle_clean();
		up(&ipm_sem);
		return -EFAULT;
	}

#ifdef DEBUG_IDLE_COUNT
	idle_cnt = 0;
	idle_ticks = 0;
#endif

	/*  Let the pointer point to ipm_start_pmu function.    */
	pipm_start_pmu = ipm_start_profiler;
	pr_info("Initialize IPM performance perfiler.\n");
	ipm_thread_exit = 0;
	init_completion(&ipm_thread_over);

	/*	Should we start the kernel thread now?	*/
	ret = kernel_thread(ipm_thread, NULL , 0);

	return 0;
}

/* Exit the PMU profiler	*/
static void __exit ipm_exit(void)
{
	/* Remove procfs */
	ipm_proc_cleanup();

	ipm_idle_clean();

	pipm_start_pmu = NULL;
	pr_info("Quit IPM profilers\n");
	ipm_thread_exit = 1;
	up(&ipm_sem);
	wait_for_completion(&ipm_thread_over);

}

module_init(ipm_init);
module_exit(ipm_exit);

MODULE_DESCRIPTION("IPM Profiler");
MODULE_LICENSE("GPL");

