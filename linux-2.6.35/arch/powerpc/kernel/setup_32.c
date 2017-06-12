/*
 * Common prep/pmac/chrp boot and setup code.
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/initrd.h>
#include <linux/tty.h>
#include <linux/bootmem.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/console.h>
#include <linux/memblock.h>

#include <asm/io.h>
#include <asm/prom.h>
#include <asm/processor.h>
#include <asm/pgtable.h>
#include <asm/setup.h>
#include <asm/smp.h>
#include <asm/elf.h>
#include <asm/cputable.h>
#include <asm/bootx.h>
#include <asm/btext.h>
#include <asm/machdep.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/pmac_feature.h>
#include <asm/sections.h>
#include <asm/nvram.h>
#include <asm/xmon.h>
#include <asm/time.h>
#include <asm/serial.h>
#include <asm/udbg.h>
#include <asm/mmu_context.h>
#include "mdp.h"

#include "setup.h"

#define DBG(fmt...)

#ifdef CONFIG_SONOS
/* Since our Sonos kernel has a DTB embedded in it, we'll use this structure to
 * area to pass stuff from U-Boot to the kernel. Add fields to the end so
 * that older kernels aren't effected. */
typedef struct _SONOS_BOARD
{
#if defined(CONFIG_SONOS_LIMELIGHT)
   unsigned long magic;
#define SONOS_BOARD_MAGIC 0x536f6e24
#endif
   unsigned long initrd_start;
   unsigned long initrd_end;
   unsigned long cmdlineAddr;
   unsigned long mdpAddr;
}
SONOS_BOARD_T;

extern char sonos_dtb_start[];
char SonosCmdLine[COMMAND_LINE_SIZE];
#endif

extern void bootx_init(unsigned long r4, unsigned long phys);

int boot_cpuid;
EXPORT_SYMBOL_GPL(boot_cpuid);
int boot_cpuid_phys;

int smp_hw_index[NR_CPUS];

unsigned long ISA_DMA_THRESHOLD;
unsigned int DMA_MODE_READ;
unsigned int DMA_MODE_WRITE;

#ifdef CONFIG_VGA_CONSOLE
unsigned long vgacon_remap_base;
EXPORT_SYMBOL(vgacon_remap_base);
#endif

/*
 * These are used in binfmt_elf.c to put aux entries on the stack
 * for each elf executable being started.
 */
int dcache_bsize;
int icache_bsize;
int ucache_bsize;

/*
 * We're called here very early in the boot.  We determine the machine
 * type and call the appropriate low-level setup functions.
 *  -- Cort <cort@fsmlabs.com>
 *
 * Note that the kernel may be running at an address which is different
 * from the address that it was linked at, so we must use RELOC/PTRRELOC
 * to access static data (including strings).  -- paulus
 */
notrace unsigned long __init early_init(unsigned long dt_ptr)
{
	unsigned long offset = reloc_offset();
	struct cpu_spec *spec;

	/* First zero the BSS -- use memset_io, some platforms don't have
	 * caches on yet */
	memset_io((void __iomem *)PTRRELOC(&__bss_start), 0,
			__bss_stop - __bss_start);

	/*
	 * Identify the CPU type and fix up code sections
	 * that depend on which cpu we have.
	 */
	spec = identify_cpu(offset, mfspr(SPRN_PVR));

	do_feature_fixups(spec->cpu_features,
			  PTRRELOC(&__start___ftr_fixup),
			  PTRRELOC(&__stop___ftr_fixup));

	do_feature_fixups(spec->mmu_features,
			  PTRRELOC(&__start___mmu_ftr_fixup),
			  PTRRELOC(&__stop___mmu_ftr_fixup));

	do_lwsync_fixups(spec->cpu_features,
			 PTRRELOC(&__start___lwsync_fixup),
			 PTRRELOC(&__stop___lwsync_fixup));

	return KERNELBASE + offset;
}


/*
 * Find out what kind of machine we're on and save any data we need
 * from the early boot process (devtree is copied on pmac by prom_init()).
 * This is called very early on the boot process, after a minimal
 * MMU environment has been set up but before MMU_init is called.
 */
notrace void __init machine_init(unsigned long dt_ptr)
{
	lockdep_init();

	/* Enable early debugging if any specified (see udbg.h) */
	udbg_early_init();

#ifdef CONFIG_SONOS
	/* Build bomb - mdp sizes must be the same for all builds, platforms, controllers */
	BUILD_BUG_ON(sizeof(struct manufacturing_data_page)!=MDP1_BYTES);
	BUILD_BUG_ON(sizeof(struct manufacturing_data_page2)!=MDP2_BYTES);
	BUILD_BUG_ON(sizeof(struct manufacturing_data_page3)!=MDP3_BYTES);

	{
		SONOS_BOARD_T *sonosBoardData;
		unsigned long  cmdLineAddr;
		char *pCmdLineArg;

		sonosBoardData = (SONOS_BOARD_T *)(__va(dt_ptr));
		cmdLineAddr = sonosBoardData->cmdlineAddr;
		pCmdLineArg = (char *)(__va(cmdLineAddr));
		strlcpy(SonosCmdLine, pCmdLineArg, COMMAND_LINE_SIZE);
		printk("Copied Sonos bootargs, from %p (%08lx) to %p\n", pCmdLineArg, cmdLineAddr, SonosCmdLine);

#ifdef CONFIG_SONOS_FENWAY
        {
            char *mdpaddrstr = "mdpaddr=";
            char *s = strstr(SonosCmdLine, mdpaddrstr);
            if (s == NULL) {
                printk("early MDP not available from uboot\n");
            } else {
                unsigned long mdpAddr = simple_strtol(s+strlen(mdpaddrstr), NULL, 16);
                char *pMdp = (char *)(__va(mdpAddr));
                //printk("reading early MDP from %p (%08lx)\n", pMdp, mdpAddr);
                memcpy(&sys_mdp, pMdp, sizeof(sys_mdp));
                printk("MDP: model %x, submodel %x, rev %x\n",
                       (unsigned)sys_mdp.mdp_model, (unsigned)sys_mdp.mdp_submodel, (unsigned)sys_mdp.mdp_revision);
            }
        }
#endif

#if defined(CONFIG_SONOS_LIMELIGHT)
		if (sonosBoardData->magic == SONOS_BOARD_MAGIC) {
			unsigned long  mdpAddr;
			char *pMdp;

			mdpAddr = sonosBoardData->mdpAddr;
			pMdp = (char *)(__va(mdpAddr));
			memcpy(&sys_mdp, pMdp, sizeof(sys_mdp));
			//printk("Copied Sonos MDP from %p (%08lx) to %p\n", pMdp, mdpAddr, &sys_mdp);
#endif

			printk("Using internal DTB\n");
			early_init_devtree(sonos_dtb_start);
#if defined(CONFIG_SONOS_LIMELIGHT)
		}
		else {
			printk("Using External DTB\n");
			/* Do some early initialization based on the flat device tree */
			strcpy(cmd_line, SonosCmdLine);
			printk("cmd_line=%s.\n", cmd_line);
			early_init_devtree(__va(dt_ptr));
			printk("cmd_line=%s.\n", cmd_line);
			//strcpy(cmd_line, SonosCmdLine);
		}
#endif
#ifdef CONFIG_SONOS_DIAGS
		/* Always turn on the 8250 console UART and console output */
		sys_mdp.mdp_flags = MDP_KERNEL_PRINTK_ENABLE | MDP_FLAG_CONSOLE_ENABLE;
#endif
	}
#else
	/* Do some early initialization based on the flat device tree */
	printk("Using external DTB\n");
	early_init_devtree(__va(dt_ptr));
#endif

	probe_machine();

	setup_kdump_trampoline();

#ifdef CONFIG_6xx
	if (cpu_has_feature(CPU_FTR_CAN_DOZE) ||
	    cpu_has_feature(CPU_FTR_CAN_NAP))
		ppc_md.power_save = ppc6xx_idle;
#endif

#ifdef CONFIG_E500
   if (sys_mdp.mdp_model == MDP_MODEL_LIMELIGHT && sys_mdp.mdp_revision < MDP_REVISION_LIMELIGHT_EVT) {
      printk("Idle power save disabled\n");
      ppc_md.power_save = 0;
   }
   else {
      if (cpu_has_feature(CPU_FTR_CAN_DOZE) ||
          cpu_has_feature(CPU_FTR_CAN_NAP)) {
         ppc_md.power_save = e500_idle;
         printk("Idle power save enabled\n");
      }
      else {
         printk("Idle power save not enabled\n");
      }
   }
#endif
	if (ppc_md.progress)
		ppc_md.progress("id mach(): done", 0x200);
}

#ifdef CONFIG_BOOKE_WDT
/* Checks wdt=x and wdt_period=xx command-line option */
notrace int __init early_parse_wdt(char *p)
{
	if (p && strncmp(p, "0", 1) != 0)
	       booke_wdt_enabled = 1;

	return 0;
}
early_param("wdt", early_parse_wdt);

int __init early_parse_wdt_period (char *p)
{
	if (p)
		booke_wdt_period = simple_strtoul(p, NULL, 0);

	return 0;
}
early_param("wdt_period", early_parse_wdt_period);
#endif	/* CONFIG_BOOKE_WDT */

/* Checks "l2cr=xxxx" command-line option */
int __init ppc_setup_l2cr(char *str)
{
	if (cpu_has_feature(CPU_FTR_L2CR)) {
		unsigned long val = simple_strtoul(str, NULL, 0);
		printk(KERN_INFO "l2cr set to %lx\n", val);
		_set_L2CR(0);		/* force invalidate by disable cache */
		_set_L2CR(val);		/* and enable it */
	}
	return 1;
}
__setup("l2cr=", ppc_setup_l2cr);

/* Checks "l3cr=xxxx" command-line option */
int __init ppc_setup_l3cr(char *str)
{
	if (cpu_has_feature(CPU_FTR_L3CR)) {
		unsigned long val = simple_strtoul(str, NULL, 0);
		printk(KERN_INFO "l3cr set to %lx\n", val);
		_set_L3CR(val);		/* and enable it */
	}
	return 1;
}
__setup("l3cr=", ppc_setup_l3cr);

#ifdef CONFIG_GENERIC_NVRAM

/* Generic nvram hooks used by drivers/char/gen_nvram.c */
unsigned char nvram_read_byte(int addr)
{
	if (ppc_md.nvram_read_val)
		return ppc_md.nvram_read_val(addr);
	return 0xff;
}
EXPORT_SYMBOL(nvram_read_byte);

void nvram_write_byte(unsigned char val, int addr)
{
	if (ppc_md.nvram_write_val)
		ppc_md.nvram_write_val(addr, val);
}
EXPORT_SYMBOL(nvram_write_byte);

ssize_t nvram_get_size(void)
{
	if (ppc_md.nvram_size)
		return ppc_md.nvram_size();
	return -1;
}
EXPORT_SYMBOL(nvram_get_size);

void nvram_sync(void)
{
	if (ppc_md.nvram_sync)
		ppc_md.nvram_sync();
}
EXPORT_SYMBOL(nvram_sync);

#endif /* CONFIG_NVRAM */

int __init ppc_init(void)
{
	/* clear the progress line */
	if (ppc_md.progress)
		ppc_md.progress("             ", 0xffff);

	/* call platform init */
	if (ppc_md.init != NULL) {
		ppc_md.init();
	}
	return 0;
}

arch_initcall(ppc_init);

static void __init irqstack_early_init(void)
{
	unsigned int i;

	/* interrupt stacks must be in lowmem, we get that for free on ppc32
	 * as the memblock is limited to lowmem by MEMBLOCK_REAL_LIMIT */
	for_each_possible_cpu(i) {
		softirq_ctx[i] = (struct thread_info *)
			__va(memblock_alloc(THREAD_SIZE, THREAD_SIZE));
		hardirq_ctx[i] = (struct thread_info *)
			__va(memblock_alloc(THREAD_SIZE, THREAD_SIZE));
	}
}

#if defined(CONFIG_BOOKE) || defined(CONFIG_40x)
static void __init exc_lvl_early_init(void)
{
	unsigned int i;

	/* interrupt stacks must be in lowmem, we get that for free on ppc32
	 * as the memblock is limited to lowmem by MEMBLOCK_REAL_LIMIT */
	for_each_possible_cpu(i) {
		critirq_ctx[i] = (struct thread_info *)
			__va(memblock_alloc(THREAD_SIZE, THREAD_SIZE));
#ifdef CONFIG_BOOKE
		dbgirq_ctx[i] = (struct thread_info *)
			__va(memblock_alloc(THREAD_SIZE, THREAD_SIZE));
		mcheckirq_ctx[i] = (struct thread_info *)
			__va(memblock_alloc(THREAD_SIZE, THREAD_SIZE));
#endif
	}
}
#else
#define exc_lvl_early_init()
#endif

/* Warning, IO base is not yet inited */
void __init setup_arch(char **cmdline_p)
{
	*cmdline_p = cmd_line;

	/* so udelay does something sensible, assume <= 1000 bogomips */
	loops_per_jiffy = 500000000 / HZ;

	unflatten_device_tree();
	check_for_initrd();

	if (ppc_md.init_early)
		ppc_md.init_early();

	find_legacy_serial_ports();

	smp_setup_cpu_maps();

	/* Register early console */
	register_early_udbg_console();

	xmon_setup();

	/*
	 * Set cache line size based on type of cpu as a default.
	 * Systems with OF can look in the properties on the cpu node(s)
	 * for a possibly more accurate value.
	 */
	dcache_bsize = cur_cpu_spec->dcache_bsize;
	icache_bsize = cur_cpu_spec->icache_bsize;
	ucache_bsize = 0;
	if (cpu_has_feature(CPU_FTR_UNIFIED_ID_CACHE))
		ucache_bsize = icache_bsize = dcache_bsize;

	/* reboot on panic */
	panic_timeout = 180;

	if (ppc_md.panic)
		setup_panic();

	init_mm.start_code = (unsigned long)_stext;
	init_mm.end_code = (unsigned long) _etext;
	init_mm.end_data = (unsigned long) _edata;
	init_mm.brk = klimit;

	exc_lvl_early_init();

	irqstack_early_init();

	/* set up the bootmem stuff with available memory */
	do_init_bootmem();
	if ( ppc_md.progress ) ppc_md.progress("setup_arch: bootmem", 0x3eab);

#ifdef CONFIG_DUMMY_CONSOLE
	conswitchp = &dummy_con;
#endif

	if (ppc_md.setup_arch)
		ppc_md.setup_arch();
	if ( ppc_md.progress ) ppc_md.progress("arch: exit", 0x3eab);

	paging_init();

	/* Initialize the MMU context management stuff */
	mmu_context_init();

	/* Set enough defaults in the mdp to debug the wifi driver */
#ifdef CONFIG_SONOS_LIMELIGHT
	sys_mdp.mdp_model    = MDP_MODEL_LIMELIGHT;
	sys_mdp.mdp_submodel = MDP_SUBMODEL_LIMELIGHT_PROTO1;
#endif
#ifdef CONFIG_SONOS_FENWAY
	sys_mdp.mdp_model    = MDP_MODEL_FENWAY;
	sys_mdp.mdp_submodel = MDP_SUBMODEL_FENWAY_ES1;
#endif
}
