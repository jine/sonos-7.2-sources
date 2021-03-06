/*
 *   fs/cifs/dir.c
 *
 *   vfs operations that deal with dentries
 * 
 *   Copyright (C) International Business Machines  Corp., 2002,2003
 *   Author(s): Steve French (sfrench@us.ibm.com)
 *
 *   This library is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published
 *   by the Free Software Foundation; either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include "cifsfs.h"
#include "cifspdu.h"
#include "cifsglob.h"
#include "cifsproto.h"
#include "cifs_debug.h"
#include "cifs_fs_sb.h"

void
renew_parental_timestamps(struct dentry *direntry)
{
	/* BB check if there is a way to get the kernel to do this or if we really need this */
	do {
		direntry->d_time = jiffies;
		direntry = direntry->d_parent;
	} while (!IS_ROOT(direntry));	
}

/* Note: caller must free return buffer */
char *
build_path_from_dentry(struct dentry *direntry)
{
	struct dentry *temp;
	int namelen = 0;
	char *full_path;

	if(direntry == NULL)
		return NULL;  /* not much we can do if dentry is freed and
		we need to reopen the file after it was closed implicitly
		when the server crashed */

cifs_bp_rename_retry:
	for (temp = direntry; !IS_ROOT(temp);) {
		namelen += (1 + temp->d_name.len);
		temp = temp->d_parent;
		if(temp == NULL) {
			cERROR(1,("corrupt dentry"));
			return NULL;
		}
	}

	full_path = kmalloc(namelen+1, GFP_KERNEL);
	if(full_path == NULL)
		return full_path;
	full_path[namelen] = 0;	/* trailing null */

	for (temp = direntry; !IS_ROOT(temp);) {
		namelen -= 1 + temp->d_name.len;
		if (namelen < 0) {
			break;
		} else {
			full_path[namelen] = '\\';
			strncpy(full_path + namelen + 1, temp->d_name.name,
				temp->d_name.len);
			cFYI(0, (" name: %s ", full_path + namelen));
		}
		temp = temp->d_parent;
		if(temp == NULL) {
			cERROR(1,("corrupt dentry"));
			kfree(full_path);
			return NULL;
		}
	}
	if (namelen != 0) {
		cERROR(1,
		       ("We did not end path lookup where we expected namelen is %d",
			namelen));
		/* presumably this is only possible if we were racing with a rename 
		of one of the parent directories  (we can not lock the dentries
		above us to prevent this, but retrying should be harmless) */
		kfree(full_path);
		namelen = 0;
		goto cifs_bp_rename_retry;
	}

	return full_path;
}

char *
build_wildcard_path_from_dentry(struct dentry *direntry)
{
	struct dentry *temp;
	int namelen = 0;
	char *full_path;

	for (temp = direntry; !IS_ROOT(temp);) {
		namelen += (1 + temp->d_name.len);
		temp = temp->d_parent;
	}
	namelen += 3;		/* allow for trailing null and wildcard (slash and *) */
	full_path = kmalloc(namelen, GFP_KERNEL);
	namelen--;
	full_path[namelen] = 0;	/* trailing null */
	namelen--;
	full_path[namelen] = '*';
	namelen--;
	full_path[namelen] = '\\';

	for (temp = direntry; !IS_ROOT(temp);) {
		namelen -= 1 + temp->d_name.len;
		if (namelen < 0) {
			break;
		} else {
			full_path[namelen] = '\\';
			strncpy(full_path + namelen + 1, temp->d_name.name,
				temp->d_name.len);
		}
		temp = temp->d_parent;
	}
	if (namelen != 0)
		cERROR(1,
		       ("We did not end path lookup where we expected namelen is %d",
			namelen));

	return full_path;
}

/* Inode operations in similar order to how they appear in the Linux file fs.h */

int
cifs_create(struct inode *inode, struct dentry *direntry, int mode)
{
	int rc = -ENOENT;
	int xid;
	int oplock = 0; /* no sense requested oplock if we are just going to
			 immediately  close the file */
	__u16 fileHandle;
	struct cifs_sb_info *cifs_sb;
	struct cifsTconInfo *pTcon;
	char *full_path = NULL;
	FILE_ALL_INFO * buf = NULL;
	struct inode *newinode = NULL;

	xid = GetXid();

	cifs_sb = CIFS_SB(inode->i_sb);
	pTcon = cifs_sb->tcon;

	down(&direntry->d_sb->s_vfs_rename_sem);
	full_path = build_path_from_dentry(direntry);
	up(&direntry->d_sb->s_vfs_rename_sem);
	if(full_path == NULL) {
		FreeXid(xid);
		return -ENOMEM;
	}

	/* BB add processing to set equivalent of mode - e.g. via CreateX with ACLs */

        buf = kmalloc(sizeof(FILE_ALL_INFO),GFP_KERNEL);
	rc = CIFSSMBOpen(xid, pTcon, full_path, FILE_OVERWRITE_IF,
			GENERIC_WRITE, CREATE_NOT_DIR,
			&fileHandle, &oplock, buf, cifs_sb->local_nls);
	if (rc) {
		cFYI(1, ("cifs_create returned 0x%x ", rc));
	} else {
	/* BB for case of overwriting existing file can we use the inode that was
		 passed in rather than creating new one?? */
		if (pTcon->ses->capabilities & CAP_UNIX)
			rc = cifs_get_inode_info_unix(&newinode, full_path,
						      inode->i_sb);
		else
			rc = cifs_get_inode_info(&newinode, full_path,
						 buf, inode->i_sb);

		if (rc != 0) {
			cFYI(1,("Create worked but get_inode_info failed with rc = %d",
			      rc));
		} else {
			direntry->d_op = &cifs_dentry_ops;
			d_instantiate(direntry, newinode);
		}
		CIFSSMBClose(xid, pTcon, fileHandle);

		if(newinode) {
			newinode->i_mode = mode;
			if (cifs_sb->tcon->ses->capabilities & CAP_UNIX)                
				CIFSSMBUnixSetPerms(xid, pTcon, full_path, inode->i_mode,
					(__u64)-1, 
					(__u64)-1,
					0 /* dev */,
					cifs_sb->local_nls);
			else { /* BB implement via Windows security descriptors */
			/* eg CIFSSMBWinSetPerms(xid,pTcon,full_path,mode,-1,-1,local_nls);*/
			/* in the meantime could set r/o dos attribute when perms are eg:
				mode & 0222 == 0 */
			}
		}
	} 

	if (buf)
	    kfree(buf);
	if (full_path)
	    kfree(full_path);
	FreeXid(xid);

	return rc;
}

int cifs_mknod(struct inode *inode, struct dentry *direntry, int mode, int device_number) 
{
	int rc = -EPERM;
	int xid;
	struct cifs_sb_info *cifs_sb;
	struct cifsTconInfo *pTcon;
	char *full_path = NULL;
	struct inode * newinode = NULL;

	xid = GetXid();

	cifs_sb = CIFS_SB(inode->i_sb);
	pTcon = cifs_sb->tcon;

	down(&direntry->d_sb->s_vfs_rename_sem);
	full_path = build_path_from_dentry(direntry);
	up(&direntry->d_sb->s_vfs_rename_sem);
	if(full_path == NULL)
		rc = -ENOMEM;
	
	if (full_path && (pTcon->ses->capabilities & CAP_UNIX)) {
		rc = CIFSSMBUnixSetPerms(xid, pTcon,
			full_path, mode, current->euid, current->egid,
			device_number, cifs_sb->local_nls);
		if(!rc) {
			rc = cifs_get_inode_info_unix(&newinode, full_path,
						inode->i_sb);
			direntry->d_op = &cifs_dentry_ops;
			if(rc == 0)
				d_instantiate(direntry, newinode);
		}
	}

	if (full_path)
		kfree(full_path);
	FreeXid(xid);

	return rc;
}


struct dentry *
cifs_lookup(struct inode *parent_dir_inode, struct dentry *direntry)
{
	int xid;
	int rc = 0; /* to get around spurious gcc warning, set to zero here */
	struct cifs_sb_info *cifs_sb;
	struct cifsTconInfo *pTcon;
	struct inode *newInode = NULL;
	char *full_path = NULL;

	xid = GetXid();

	cFYI(1,
	     (" parent inode = 0x%p name is: %s and dentry = 0x%p",
	      parent_dir_inode, direntry->d_name.name, direntry));

	/* BB Add check of incoming data - e.g. frame not longer than maximum SMB - let server check the namelen BB */

	/* check whether path exists */

	cifs_sb = CIFS_SB(parent_dir_inode->i_sb);
	pTcon = cifs_sb->tcon;

	/* can not grab the rename sem here since it would
	deadlock in the cases (beginning of sys_rename itself)
	in which we already have the sb rename sem */
	full_path = build_path_from_dentry(direntry);
	if(full_path == NULL) {
		FreeXid(xid);
		return ERR_PTR(-ENOMEM);
	}

	if (direntry->d_inode != NULL) {
		cFYI(1, (" non-NULL inode in lookup"));
	} else {
		cFYI(1, (" NULL inode in lookup"));
	}
	cFYI(1,
	     (" Full path: %s inode = 0x%p", full_path, direntry->d_inode));

	if (pTcon->ses->capabilities & CAP_UNIX)
		rc = cifs_get_inode_info_unix(&newInode, full_path,
					      parent_dir_inode->i_sb);
	else
		rc = cifs_get_inode_info(&newInode, full_path, NULL,
					 parent_dir_inode->i_sb);

	if ((rc == 0) && (newInode != NULL)) {
		direntry->d_op = &cifs_dentry_ops;
		d_add(direntry, newInode);

		/* since paths are not looked up by component - the parent directories are presumed to be good here */
		renew_parental_timestamps(direntry);

	} else if (rc == -ENOENT) {
		rc = 0;
		direntry->d_op = &cifs_dentry_ops;
		d_add(direntry, NULL);
		renew_parental_timestamps(direntry);
	} else {
		cERROR(1,("Error 0x%x or on cifs_get_inode_info in lookup",rc));
		/* BB special case check for Access Denied - watch security 
		exposure of returning dir info implicitly via different rc 
		if file exists or not but no access BB */
	}

	if (full_path)
		kfree(full_path);
	FreeXid(xid);
	return ERR_PTR(rc);
}

int
cifs_dir_open(struct inode *inode, struct file *file)
{				/* NB: currently unused since searches are opened in readdir */
	int rc = 0;
	int xid;
	struct cifs_sb_info *cifs_sb;
	struct cifsTconInfo *pTcon;
	char *full_path = NULL;

	xid = GetXid();

	cifs_sb = CIFS_SB(inode->i_sb);
	pTcon = cifs_sb->tcon;

	full_path = build_wildcard_path_from_dentry(file->f_dentry);

	cFYI(1, (" inode = 0x%p and full path is %s", inode, full_path));

	if (full_path)
		kfree(full_path);
	FreeXid(xid);
	return rc;
}

static int
cifs_d_revalidate(struct dentry *direntry, int flags)
{
	int isValid = 1;

/*	lock_kernel(); *//* surely we do not want to lock the kernel for a whole network round trip which could take seconds */

	if (direntry->d_inode) {
		if (cifs_revalidate(direntry)) {
			/* unlock_kernel(); */
			return 0;
		}
	} else {
		cFYI(1,
		     ("In cifs_d_revalidate with no inode but name = %s and dentry 0x%p",
		      direntry->d_name.name, direntry));
		isValid = 0;
	}

/*    unlock_kernel(); */

	return isValid;
}

/* static int cifs_d_delete(struct dentry *direntry)
{
	int rc = 0;

	cFYI(1, ("In cifs d_delete, name = %s", direntry->d_name.name));

	return rc;
}     */

struct dentry_operations cifs_dentry_ops = {
	.d_revalidate = cifs_d_revalidate,
/* d_delete:       cifs_d_delete,       *//* not needed except for debugging */
	/* no need for d_hash, d_compare, d_release, d_iput ... yet. BB confirm this BB */
};
