/*
 *   Mesh network routing
 *
 *      $Id: prochandlers.c,v 1.4 2004/01/08 19:20:43 vangool Exp $
 *
 */
#include <linux/proc_fs.h>

#include "prochandlers.h"

/* Entries for our different /proc files */
static struct proc_dir_entry *mesh_dir;           /* Proc directory... */
static struct proc_dir_entry *assoc_proc;         /* ...and its entries */
static struct proc_dir_entry *routetable_proc;
static struct proc_dir_entry *signal_proc;
static struct proc_dir_entry *stats_proc;

#ifdef CONFIG_PROC_FS
int read_assoc_proc(char *buffer, char **start, off_t offset,
	int length,int *eof,void *data)
{
    return 0;
}

int read_routetable_proc(char *buffer, char **start, off_t offset,
	int length,int *eof,void *data)
{
    return 0;
}

int read_signal_proc(char *buffer, char **start, off_t offset,
	int length,int *eof,void *data)
{
    return 0;
}

int read_stats_proc(char *buffer, char **start, off_t offset,
	int length,int *eof,void *data)
{
    return 0;
}
#endif

/* Setup the proc file system */
void proc_init(void)
{
#ifdef CONFIG_PROC_FS
    printk("Initializing proc entries\n");

    /* Setup our proc directory */
    mesh_dir = proc_mkdir("mesh", NULL);
    mesh_dir->owner = THIS_MODULE;

    /* The associated ZP (only applicable to HH) */
    assoc_proc = create_proc_read_entry("mesh/assoc", 0, NULL,
	    read_assoc_proc, NULL);
    assoc_proc->owner = THIS_MODULE;

    /* The internal routing table */
    routetable_proc = create_proc_read_entry("mesh/routetable", 0, NULL,
	    read_routetable_proc, NULL);
    routetable_proc->owner = THIS_MODULE;

    /* Signal stength */
    signal_proc = create_proc_read_entry("mesh/signal", 0, NULL,
	    read_signal_proc, NULL);
    signal_proc->owner = THIS_MODULE;

    /* Some statictics */
    stats_proc = create_proc_read_entry("mesh/stats", 0, NULL,
	    read_stats_proc, NULL);
    stats_proc->owner = THIS_MODULE;
#endif
}

/* Get rid of all our proc files */
void proc_deinit(void)
{
#ifdef CONFIG_PROC_FS
    printk("Cleaning up proc entries\n");

    remove_proc_entry("mesh/assoc", NULL);
    remove_proc_entry("mesh/routetable", NULL);
    remove_proc_entry("mesh/signal", NULL);
    remove_proc_entry("mesh/stats", NULL);
    remove_proc_entry("mesh", NULL);
#endif
}

