/* Copyright (c) 2004, Rincon Networks, Inc.  All rights reserved. */

#ifndef PTABLE_H
#define PTABLE_H

struct ptable_ent {
	unsigned int pe_type;
#define PE_TYPE_END	 0
#define PE_TYPE_DIAGS	 1
#define PE_TYPE_RESERVED 2
#define PE_TYPE_KERNEL	16
#define PE_TYPE_ROOT	17
#define PE_TYPE_OPT		18
#define PE_TYPE_JFFS		64
	int pe_start;
	int pe_nblocks;
};

struct ptable {
	unsigned int pt_magic;
#define PT_MAGIC 0x653503e4
	unsigned int pt_flags;
#define PTF_REL		1
	struct ptable_ent pt_entries[10];
	/* 8 + 120 = 128 bytes total*/
	unsigned char pt_reserved[384];
};

#endif  /* PTABLE_H */
