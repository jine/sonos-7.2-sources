/*
 * Copyright (C) 2001-2004 Sistina Software, Inc. All rights reserved.
 * Copyright (C) 2004-2012 Red Hat, Inc. All rights reserved.
 *
 * This file is part of the device-mapper userspace tools.
 *
 * This copyrighted material is made available to anyone wishing to use,
 * modify, copy, or redistribute it subject to the terms and conditions
 * of the GNU Lesser General Public License v.2.1.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "dmlib.h"
#include "libdm-targets.h"
#include "libdm-common.h"
#include "kdev_t.h"
#include "dm-ioctl.h"

#include <stdarg.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <dirent.h>

#ifdef linux
#  include <linux/fs.h>
#endif

#ifdef HAVE_SELINUX
#  include <selinux/selinux.h>
#endif
#ifdef HAVE_SELINUX_LABEL_H
#  include <selinux/label.h>
#endif

#include "sonos_common.h"

#define DM_DEFAULT_NAME_MANGLING_MODE_ENV_VAR_NAME "DM_DEFAULT_NAME_MANGLING_MODE"

#define DEV_DIR "/dev/"

static char _dm_dir[PATH_MAX] = DEV_DIR DM_DIR;
static char _sysfs_dir[PATH_MAX] = "/sys/";
static char _path0[PATH_MAX];           /* path buffer, safe 4kB on stack */

static int _verbose = 0;
static int _suspended_dev_counter = 0;
static dm_string_mangling_t _name_mangling_mode = DEFAULT_DM_NAME_MANGLING;

#ifdef HAVE_SELINUX_LABEL_H
static struct selabel_handle *_selabel_handle = NULL;
#endif

#define DM_DEVICE_MAJOR 252

void dm_log_init_verbose(int level)
{
	_verbose = level;
}

static void _build_dev_path(char *buffer, size_t len, const char *dev_name)
{
	/* If there's a /, assume caller knows what they're doing */
	if (strchr(dev_name, '/'))
		snprintf(buffer, len, "%s", dev_name);
	else
		snprintf(buffer, len, "%s/%s", _dm_dir, dev_name);
}

int dm_get_library_version(char *version, size_t size)
{
	strncpy(version, DM_LIB_VERSION, size);
	return 1;
}

int dm_get_suspended_counter(void)
{
	return _suspended_dev_counter;
}

int dm_set_name_mangling_mode(dm_string_mangling_t name_mangling_mode)
{
	_name_mangling_mode = name_mangling_mode;

	return 1;
}

dm_string_mangling_t dm_get_name_mangling_mode(void)
{
	return _name_mangling_mode;
}

struct dm_task *dm_task_create(int type)
{
	struct dm_task *dmt = dm_zalloc(sizeof(*dmt));

	if (!dmt) {
		syslog(LOG_INFO, "dm_task_create: malloc(%" PRIsize_t ") failed",
			  sizeof(*dmt));
		return NULL;
	}

	if (!dm_check_version()) {
		dm_free(dmt);
		return_NULL;
	}

	dmt->type = type;
	dmt->minor = 10;
	dmt->major = DM_DEVICE_MAJOR;
	if ( type == DM_DEVICE_STATUS || type == DM_DEVICE_TABLE ||
		type == DM_DEVICE_RELOAD ||
		type == DM_DEVICE_REMOVE ) {
		dmt->minor = -1;
		dmt->major = -1;
	}
	dmt->allow_default_major_fallback = 1;
	dmt->uid = DM_DEVICE_UID;
	dmt->gid = DM_DEVICE_GID;
	dmt->mode = DM_DEVICE_MODE;
	dmt->no_open_count = 0;
	dmt->read_ahead = DM_READ_AHEAD_AUTO;
	dmt->read_ahead_flags = 0;
	dmt->event_nr = 0;
	dmt->cookie_set = 0;
	dmt->query_inactive_table = 0;
	dmt->new_uuid = 0;
	dmt->secure_data = 0;

	return dmt;
}

/*
 * Find the name associated with a given device number by scanning _dm_dir.
 */
static int _find_dm_name_of_device(dev_t st_rdev, char *buf, size_t buf_len)
{
	const char *name;
	char path[PATH_MAX];
	struct dirent *dirent;
	DIR *d;
	struct stat st;
	int r = 0;

	if (!(d = opendir(_dm_dir))) {
		log_sys_error("opendir", _dm_dir);
		return 0;
	}

	while ((dirent = readdir(d))) {
		name = dirent->d_name;

		if (!strcmp(name, ".") || !strcmp(name, ".."))
			continue;

		if (dm_snprintf(path, sizeof(path), "%s/%s", _dm_dir,
				name) == -1) {
			syslog(LOG_INFO, "Couldn't create path for %s", name);
			continue;
		}

		if (stat(path, &st))
			continue;

		if (st.st_rdev == st_rdev) {
			strncpy(buf, name, buf_len);
			r = 1;
			break;
		}
	}

	if (closedir(d))
		log_sys_error("closedir", _dm_dir);

	return r;
}

static int _is_whitelisted_char(char c)
{
	/*
	 * Actually, DM supports any character in a device name.
	 * This whitelist is just for proper integration with udev.
	 */
        if ((c >= '0' && c <= '9') ||
            (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            strchr("#+-.:=@_", c) != NULL)
                return 1;

        return 0;
}

int check_multiple_mangled_string_allowed(const char *str, const char *str_name,
					 dm_string_mangling_t mode)
{
	if (mode == DM_STRING_MANGLING_AUTO && strstr(str, "\\x5cx")) {
		syslog(LOG_INFO, "The %s \"%s\" seems to be mangled more than once. "
			  "This is not allowed in auto mode.", str_name, str);
		return 0;
	}

	return 1;
}

/*
 * Mangle all characters in the input string which are not on a whitelist
 * with '\xNN' format where NN is the hex value of the character.
 */
int mangle_string(const char *str, const char *str_name, size_t len,
		  char *buf, size_t buf_len, dm_string_mangling_t mode)
{
	int need_mangling = -1; /* -1 don't know yet, 0 no, 1 yes */
	size_t i, j;

	if (!str || !buf)
		return -1;

	/* Is there anything to do at all? */
	if (!*str || !len)
		return 0;

	if (buf_len < DM_NAME_LEN) {
		syslog(LOG_INFO, "mangle_string: supplied buffer too small");
		return -1;
	}

	if (mode == DM_STRING_MANGLING_NONE)
		mode = DM_STRING_MANGLING_AUTO;

	for (i = 0, j = 0; str[i]; i++) {
		if (mode == DM_STRING_MANGLING_AUTO) {
			/*
			 * Detect already mangled part of the string and keep it.
			 * Return error on mixture of mangled/not mangled!
			 */
			if (str[i] == '\\' && str[i+1] == 'x') {
				if ((len - i < 4) || (need_mangling == 1))
					goto bad1;
				if (buf_len - j < 4)
					goto bad2;

				memcpy(&buf[j], &str[i], 4);
				i+=3; j+=4;

				need_mangling = 0;
				continue;
			}
		}

		if (_is_whitelisted_char(str[i])) {
			/* whitelisted, keep it. */
			if (buf_len - j < 1)
				goto bad2;
			buf[j] = str[i];
			j++;
		} else {
			/*
			 * Not on a whitelist, mangle it.
			 * Return error on mixture of mangled/not mangled
			 * unless a DM_STRING_MANGLING_HEX is used!.
			 */
			if ((mode != DM_STRING_MANGLING_HEX) && (need_mangling == 0))
				goto bad1;
			if (buf_len - j < 4)
				goto bad2;

			sprintf(&buf[j], "\\x%02x", (unsigned char) str[i]);
			j+=4;

			need_mangling = 1;
		}
	}

	if (buf_len - j < 1)
		goto bad2;
	buf[j] = '\0';

	/* All chars in the string whitelisted? */
	if (need_mangling == -1)
		need_mangling = 0;

	return need_mangling;

bad1:
	syslog(LOG_INFO, "The %s \"%s\" contains mixed mangled and unmangled "
		  "characters or it's already mangled improperly.", str_name, str);
	return -1;
bad2:
	syslog(LOG_INFO, "Mangled form of the %s too long for \"%s\".", str_name, str);
	return -1;
}

/*
 * Try to unmangle supplied string.
 * Return value: -1 on error, 0 when no unmangling needed, 1 when unmangling applied
 */
int unmangle_string(const char *str, const char *str_name, size_t len,
		    char *buf, size_t buf_len, dm_string_mangling_t mode)
{
	int strict = mode != DM_STRING_MANGLING_NONE;
	char str_rest[DM_NAME_LEN];
	size_t i, j;
	int code;
	int r = 0;

	if (!str || !buf)
		return -1;

	/* Is there anything to do at all? */
	if (!*str || !len)
		return 0;

	if (buf_len < DM_NAME_LEN) {
		syslog(LOG_INFO, "unmangle_string: supplied buffer too small");
		return -1;
	}

	for (i = 0, j = 0; str[i]; i++, j++) {
		if (strict && !(_is_whitelisted_char(str[i]) || str[i]=='\\')) {
			syslog(LOG_INFO, "The %s \"%s\" should be mangled but "
				  "it contains blacklisted characters.", str_name, str);
			j=0; r=-1;
			goto out;
		}

		if (str[i] == '\\' && str[i+1] == 'x') {
			if (!sscanf(&str[i+2], "%2x%s", &code, str_rest)) {
				syslog(LOG_INFO, "Hex encoding mismatch detected in %s \"%s\" "
					  "while trying to unmangle it.", str_name, str);
				goto out;
			}
			buf[j] = (unsigned char) code;

			/* skip the encoded part we've just decoded! */
			i+= 3;

			/* unmangling applied */
			r = 1;
		} else
			buf[j] = str[i];
	}

out:
	buf[j] = '\0';
	return r;
}

static int _dm_task_set_name(struct dm_task *dmt, const char *name,
			     dm_string_mangling_t mangling_mode)
{
	char mangled_name[DM_NAME_LEN];
	int r = 0;

	dm_free(dmt->dev_name);
	dmt->dev_name = NULL;
	dm_free(dmt->mangled_dev_name);
	dmt->mangled_dev_name = NULL;

	if (strlen(name) >= DM_NAME_LEN) {
		syslog(LOG_INFO, "Name \"%s\" too long.", name);
		return 0;
	}

	if (!check_multiple_mangled_string_allowed(name, "name", mangling_mode))
		return_0;

	if (mangling_mode != DM_STRING_MANGLING_NONE &&
	    (r = mangle_string(name, "name", strlen(name), mangled_name,
			       sizeof(mangled_name), mangling_mode)) < 0) {
		syslog(LOG_INFO, "Failed to mangle device name \"%s\".", name);
		return 0;
	}

	/* Store mangled_dev_name only if it differs from dev_name! */
	if (r) {
		syslog(LOG_INFO, "Device name mangled [%s]: %s --> %s",
			  mangling_mode == DM_STRING_MANGLING_AUTO ? "auto" : "hex",
			  name, mangled_name);
		if (!(dmt->mangled_dev_name = dm_strdup(mangled_name))) {
			syslog(LOG_INFO, "_dm_task_set_name: dm_strdup(%s) failed", mangled_name);
			return 0;
		}
	}

	if (!(dmt->dev_name = dm_strdup(name))) {
		syslog(LOG_INFO, "_dm_task_set_name: strdup(%s) failed", name);
		return 0;
	}

	return 1;
}

static int _dm_task_set_name_from_path(struct dm_task *dmt, const char *path,
				       const char *name)
{
	char buf[PATH_MAX];
	struct stat st1, st2;
	const char *final_name;

	if (dmt->type == DM_DEVICE_CREATE) {
		syslog(LOG_INFO, "Name \"%s\" invalid. It contains \"/\".", path);
		return 0;
	}

	if (stat(path, &st1)) {
		syslog(LOG_INFO, "Device %s not found", path);
		return 0;
	}

	/*
	 * If supplied path points to same device as last component
	 * under /dev/mapper, use that name directly.  Otherwise call
	 * _find_dm_name_of_device() to scan _dm_dir for a match.
	 */
	if (dm_snprintf(buf, sizeof(buf), "%s/%s", _dm_dir, name) == -1) {
		syslog(LOG_INFO, "Couldn't create path for %s", name);
		return 0;
	}

	if (!stat(buf, &st2) && (st1.st_rdev == st2.st_rdev))
		final_name = name;
	else if (_find_dm_name_of_device(st1.st_rdev, buf, sizeof(buf)))
		final_name = buf;
	else {
		syslog(LOG_INFO, "Device %s not found", name);
		return 0;
	}

	/* This is an already existing path - do not mangle! */
	return _dm_task_set_name(dmt, final_name, DM_STRING_MANGLING_NONE);
}

int dm_task_set_name(struct dm_task *dmt, const char *name)
{
	char *pos;

	/* Path supplied for existing device? */
	if ((pos = strrchr(name, '/')))
		return _dm_task_set_name_from_path(dmt, name, pos + 1);

	return _dm_task_set_name(dmt, name, dm_get_name_mangling_mode());
}

const char *dm_task_get_name(const struct dm_task *dmt)
{
	return (dmt->dmi.v4->name);
}

static char *_task_get_string_mangled(const char *str, const char *str_name,
				      char *buf, size_t buf_size,
				      dm_string_mangling_t mode)
{
	char *rs;
	int r;

	if ((r = mangle_string(str, str_name, strlen(str), buf, buf_size, mode)) < 0)
		return NULL;

	if (!(rs = r ? dm_strdup(buf) : dm_strdup(str)))
		syslog(LOG_INFO, "_task_get_string_mangled: dm_strdup failed");

	return rs;
}

static char *_task_get_string_unmangled(const char *str, const char *str_name,
					char *buf, size_t buf_size,
					dm_string_mangling_t mode)
{
	char *rs;
	int r = 0;

	/*
	 * Unless the mode used is 'none', the string
	 * is *already* unmangled on ioctl return!
	 */
	if (mode == DM_STRING_MANGLING_NONE &&
	    (r = unmangle_string(str, str_name, strlen(str), buf, buf_size, mode)) < 0)
		return NULL;

	if (!(rs = r ? dm_strdup(buf) : dm_strdup(str)))
		syslog(LOG_INFO, "_task_get_string_unmangled: dm_strdup failed");

	return rs;
}

char *dm_task_get_name_mangled(const struct dm_task *dmt)
{
	const char *s = dm_task_get_name(dmt);
	char buf[DM_NAME_LEN];
	char *rs;

	if (!(rs = _task_get_string_mangled(s, "name", buf, sizeof(buf), dm_get_name_mangling_mode())))
		syslog(LOG_INFO, "Failed to mangle device name \"%s\".", s);

	return rs;
}

char *dm_task_get_name_unmangled(const struct dm_task *dmt)
{
	const char *s = dm_task_get_name(dmt);
	char buf[DM_NAME_LEN];
	char *rs;

	if (!(rs = _task_get_string_unmangled(s, "name", buf, sizeof(buf), dm_get_name_mangling_mode())))
		syslog(LOG_INFO, "Failed to unmangle device name \"%s\".", s);

	return rs;
}

const char *dm_task_get_uuid(const struct dm_task *dmt)
{
	return (dmt->dmi.v4->uuid);
}

char *dm_task_get_uuid_mangled(const struct dm_task *dmt)
{
	const char *s = dm_task_get_uuid(dmt);
	char buf[DM_UUID_LEN];
	char *rs;

	if (!(rs = _task_get_string_mangled(s, "UUID", buf, sizeof(buf), dm_get_name_mangling_mode())))
		syslog(LOG_INFO, "Failed to mangle device uuid \"%s\".", s);

	return rs;
}

char *dm_task_get_uuid_unmangled(const struct dm_task *dmt)
{
	const char *s = dm_task_get_uuid(dmt);
	char buf[DM_UUID_LEN];
	char *rs;

	if (!(rs = _task_get_string_unmangled(s, "UUID", buf, sizeof(buf), dm_get_name_mangling_mode())))
		syslog(LOG_INFO, "Failed to unmangle device uuid \"%s\".", s);

	return rs;
}

int dm_task_set_newname(struct dm_task *dmt, const char *newname)
{
	dm_string_mangling_t mangling_mode = dm_get_name_mangling_mode();
	char mangled_name[DM_NAME_LEN];
	int r = 0;

	if (strchr(newname, '/')) {
		syslog(LOG_INFO, "Name \"%s\" invalid. It contains \"/\".", newname);
		return 0;
	}

	if (strlen(newname) >= DM_NAME_LEN) {
		syslog(LOG_INFO, "Name \"%s\" too long", newname);
		return 0;
	}

	if (!check_multiple_mangled_string_allowed(newname, "new name", mangling_mode))
		return_0;

	if (mangling_mode != DM_STRING_MANGLING_NONE &&
	    (r = mangle_string(newname, "new name", strlen(newname), mangled_name,
			       sizeof(mangled_name), mangling_mode)) < 0) {
		syslog(LOG_INFO, "Failed to mangle new device name \"%s\"", newname);
		return 0;
	}

	if (r) {
		syslog(LOG_INFO, "New device name mangled [%s]: %s --> %s",
			  mangling_mode == DM_STRING_MANGLING_AUTO ? "auto" : "hex",
			  newname, mangled_name);
		newname = mangled_name;
	}

	if (!(dmt->newname = dm_strdup(newname))) {
		syslog(LOG_INFO, "dm_task_set_newname: strdup(%s) failed", newname);
		return 0;
	}

	dmt->new_uuid = 0;

	return 1;
}

int dm_task_set_uuid(struct dm_task *dmt, const char *uuid)
{
	char mangled_uuid[DM_UUID_LEN];
	dm_string_mangling_t mangling_mode = dm_get_name_mangling_mode();
	int r = 0;

	dm_free(dmt->uuid);
	dmt->uuid = NULL;
	dm_free(dmt->mangled_uuid);
	dmt->mangled_uuid = NULL;

	if (!check_multiple_mangled_string_allowed(uuid, "UUID", mangling_mode))
		return_0;

	if (mangling_mode != DM_STRING_MANGLING_NONE &&
	    (r = mangle_string(uuid, "UUID", strlen(uuid), mangled_uuid,
			       sizeof(mangled_uuid), mangling_mode)) < 0) {
		syslog(LOG_INFO, "Failed to mangle device uuid \"%s\".", uuid);
		return 0;
	}

	if (r) {
		syslog(LOG_INFO, "Device uuid mangled [%s]: %s --> %s",
			  mangling_mode == DM_STRING_MANGLING_AUTO ? "auto" : "hex",
			  uuid, mangled_uuid);

		if (!(dmt->mangled_uuid = dm_strdup(mangled_uuid))) {
			syslog(LOG_INFO, "dm_task_set_uuid: dm_strdup(%s) failed", mangled_uuid);
			return 0;
		}
	}

	if (!(dmt->uuid = dm_strdup(uuid))) {
		syslog(LOG_INFO, "dm_task_set_uuid: strdup(%s) failed", uuid);
		return 0;
	}

	return 1;
}

int dm_task_set_major(struct dm_task *dmt, int major)
{
	dmt->major = major;
	dmt->allow_default_major_fallback = 0;

	return 1;
}

int dm_task_set_minor(struct dm_task *dmt, int minor)
{
	dmt->minor = minor;

	return 1;
}

int dm_task_set_major_minor(struct dm_task *dmt, int major, int minor,
			    int allow_default_major_fallback)
{
	dmt->major = major;
	dmt->minor = minor;
	dmt->allow_default_major_fallback = allow_default_major_fallback;

	return 1;
}

int dm_task_set_uid(struct dm_task *dmt, uid_t uid)
{
	dmt->uid = uid;

	return 1;
}

int dm_task_set_gid(struct dm_task *dmt, gid_t gid)
{
	dmt->gid = gid;

	return 1;
}

int dm_task_set_mode(struct dm_task *dmt, mode_t mode)
{
	dmt->mode = mode;

	return 1;
}

int dm_task_enable_checks(struct dm_task *dmt)
{
	dmt->enable_checks = 1;

	return 1;
}

int dm_task_add_target(struct dm_task *dmt, uint64_t start, uint64_t size,
		       const char *ttype, const char *params)
{
	struct target *t = create_target(start, size, ttype, params);
	if (!t)
		return_0;

	if (!dmt->head)
		dmt->head = dmt->tail = t;
	else {
		dmt->tail->next = t;
		dmt->tail = t;
	}

	return 1;
}

#ifdef HAVE_SELINUX
static int _selabel_lookup(const char *path, mode_t mode,
			   security_context_t *scontext)
{
#ifdef HAVE_SELINUX_LABEL_H
	if (!_selabel_handle &&
	    !(_selabel_handle = selabel_open(SELABEL_CTX_FILE, NULL, 0))) {
		syslog(LOG_INFO, "selabel_open failed: %s", strerror(errno));
		return 0;
	}

	if (selabel_lookup(_selabel_handle, scontext, path, mode)) {
		syslog(LOG_INFO, "selabel_lookup failed for %s: %s",
			  path, strerror(errno));
		return 0;
	}
#else
	if (matchpathcon(path, mode, scontext)) {
		syslog(LOG_INFO, "matchpathcon failed for %s: %s",
			  path, strerror(errno));
		return 0;
	}
#endif
	return 1;
}
#endif

void selinux_release(void)
{
#ifdef HAVE_SELINUX_LABEL_H
	if (_selabel_handle)
		selabel_close(_selabel_handle);
	_selabel_handle = NULL;
#endif
}

static int _open_dev_node(const char *dev_name)
{
	int fd = -1;
	char path[PATH_MAX];

	_build_dev_path(path, sizeof(path), dev_name);

	if ((fd = open(path, O_RDONLY, 0)) < 0)
		log_sys_error("open", path);

	return fd;
}

int get_dev_node_read_ahead(const char *dev_name, uint32_t major, uint32_t minor,
			    uint32_t *read_ahead)
{
	char buf[24];
	int len;
	int r = 1;
	int fd;
	long read_ahead_long;

	/*
	 * If we know the device number, use sysfs if we can.
	 * Otherwise use BLKRAGET ioctl.
	 */
	if (*_sysfs_dir && major != 0) {
		if (dm_snprintf(_path0, sizeof(_path0), "%sdev/block/%" PRIu32
				":%" PRIu32 "/bdi/read_ahead_kb", _sysfs_dir,
				major, minor) < 0) {
			syslog(LOG_INFO, "Failed to build sysfs_path.");
			return 0;
		}

		if ((fd = open(_path0, O_RDONLY, 0)) != -1) {
			/* Reading from sysfs, expecting number\n */
			if ((len = read(fd, buf, sizeof(buf) - 1)) < 1) {
				log_sys_error("read", _path0);
				r = 0;
			} else {
				buf[len] = 0; /* kill \n and ensure \0 */
				*read_ahead = atoi(buf) * 2;
				syslog(LOG_INFO, "%s (%d:%d): read ahead is %" PRIu32,
					  dev_name, major, minor, *read_ahead);
			}

			if (close(fd))
				log_sys_debug("close", _path0);

			return r;
		}

		log_sys_debug("open", _path0);
		/* Fall back to use dev_name */
	}

	/*
	 * Open/close dev_name may block the process
	 * (i.e. overfilled thin pool volume)
	 */
	if (!*dev_name) {
		syslog(LOG_INFO, "Empty device name passed to BLKRAGET");
		return 0;
	}

	if ((fd = _open_dev_node(dev_name)) < 0)
		return_0;

	if (ioctl(fd, BLKRAGET, &read_ahead_long)) {
		log_sys_error("BLKRAGET", dev_name);
		*read_ahead = 0;
		r = 0;
	} else {
		*read_ahead = (uint32_t) read_ahead_long;
		syslog(LOG_INFO, "%s: read ahead is %" PRIu32, dev_name, *read_ahead);
	}

	if (close(fd))
		log_sys_debug("close", dev_name);

	return r;
}

typedef enum {
	NODE_ADD,
	NODE_DEL,
	NODE_RENAME,
	NODE_READ_AHEAD,
	NUM_NODES
} node_op_t;

static DM_LIST_INIT(_node_ops);
static int _count_node_ops[NUM_NODES];

struct node_op_parms {
	struct dm_list list;
	node_op_t type;
	char *dev_name;
	uint32_t major;
	uint32_t minor;
	uid_t uid;
	gid_t gid;
	mode_t mode;
	uint32_t read_ahead;
	uint32_t read_ahead_flags;
	char *old_name;
	int warn_if_udev_failed;
	unsigned rely_on_udev;
	char names[0];
};

static void _store_str(char **pos, char **ptr, const char *str)
{
	strcpy(*pos, str);
	*ptr = *pos;
	*pos += strlen(*ptr) + 1;
}

static void _del_node_op(struct node_op_parms *nop)
{
	_count_node_ops[nop->type]--;
	dm_list_del(&nop->list);
	dm_free(nop);

}

/* Check if there is other the type of node operation stacked */
static int _other_node_ops(node_op_t type)
{
	unsigned i;

	for (i = 0; i < NUM_NODES; i++)
		if (type != i && _count_node_ops[i])
			return 1;
	return 0;
}

static void _log_node_op(const char *action_str, struct node_op_parms *nop)
{
	const char *rely = nop->rely_on_udev ? " [trust_udev]" : "" ;
	const char *verify = nop->warn_if_udev_failed ? " [verify_udev]" : "";

	switch (nop->type) {
	case NODE_ADD:
		syslog(LOG_INFO, "%s: %s NODE_ADD (%" PRIu32 ",%" PRIu32 ") %u:%u 0%o%s%s",
			  nop->dev_name, action_str, nop->major, nop->minor, nop->uid, nop->gid, nop->mode,
			  rely, verify);
		break;
	case NODE_DEL:
		syslog(LOG_INFO, "%s: %s NODE_DEL%s%s", nop->dev_name, action_str, rely, verify);
		break;
	case NODE_RENAME:
		syslog(LOG_INFO, "%s: %s NODE_RENAME to %s%s%s", nop->old_name, action_str, nop->dev_name, rely, verify);
		break;
	case NODE_READ_AHEAD:
		syslog(LOG_INFO, "%s: %s NODE_READ_AHEAD %" PRIu32 " (flags=%" PRIu32 ")%s%s",
			  nop->dev_name, action_str, nop->read_ahead, nop->read_ahead_flags, rely, verify);
		break;
	default:
		; /* NOTREACHED */
	}
}

static int _stack_node_op(node_op_t type, const char *dev_name, uint32_t major,
			  uint32_t minor, uid_t uid, gid_t gid, mode_t mode,
			  const char *old_name, uint32_t read_ahead,
			  uint32_t read_ahead_flags, int warn_if_udev_failed,
			  unsigned rely_on_udev)
{
	struct node_op_parms *nop;
	struct dm_list *noph, *nopht;
	size_t len = strlen(dev_name) + strlen(old_name) + 2;
	char *pos;

	/*
	 * Note: warn_if_udev_failed must have valid content
	 */
	if ((type == NODE_DEL) && _other_node_ops(type))
		/*
		 * Ignore any outstanding operations on the node if deleting it.
		 */
		dm_list_iterate_safe(noph, nopht, &_node_ops) {
			nop = dm_list_item(noph, struct node_op_parms);
			if (!strcmp(dev_name, nop->dev_name)) {
				_log_node_op("Unstacking", nop);
				_del_node_op(nop);
				if (!_other_node_ops(type))
					break; /* no other non DEL ops */
			}
		}
	else if ((type == NODE_ADD) && _count_node_ops[NODE_DEL])
		/*
		 * Ignore previous DEL operation on added node.
		 * (No other operations for this device then DEL could be stacked here).
		 */
		dm_list_iterate_safe(noph, nopht, &_node_ops) {
			nop = dm_list_item(noph, struct node_op_parms);
			if ((nop->type == NODE_DEL) &&
			    !strcmp(dev_name, nop->dev_name)) {
				_log_node_op("Unstacking", nop);
				_del_node_op(nop);
				break; /* no other DEL ops */
			}
		}
	else if (type == NODE_RENAME)
		/*
		 * Ignore any outstanding operations if renaming it.
		 *
		 * Currently  RENAME operation happens through 'suspend -> resume'.
		 * On 'resume' device is added with read_ahead settings, so it is
		 * safe to remove any stacked ADD, RENAME, READ_AHEAD operation
		 * There cannot be any DEL operation on the renamed device.
		 */
		dm_list_iterate_safe(noph, nopht, &_node_ops) {
			nop = dm_list_item(noph, struct node_op_parms);
			if (!strcmp(old_name, nop->dev_name)) {
				_log_node_op("Unstacking", nop);
				_del_node_op(nop);
			}
		}
	else if (type == NODE_READ_AHEAD) {
		/* udev doesn't process readahead */
		rely_on_udev = 0;
		warn_if_udev_failed = 0;
	}

	if (!(nop = dm_malloc(sizeof(*nop) + len))) {
		syslog(LOG_INFO, "Insufficient memory to stack mknod operation");
		return 0;
	}

	pos = nop->names;
	nop->type = type;
	nop->major = major;
	nop->minor = minor;
	nop->uid = uid;
	nop->gid = gid;
	nop->mode = mode;
	nop->read_ahead = read_ahead;
	nop->read_ahead_flags = read_ahead_flags;
	nop->rely_on_udev = rely_on_udev;

	/*
	 * Clear warn_if_udev_failed if rely_on_udev is set.  It doesn't get
	 * checked in this case - this just removes the flag from log messages.
	 */
	nop->warn_if_udev_failed = rely_on_udev ? 0 : warn_if_udev_failed;

	_store_str(&pos, &nop->dev_name, dev_name);
	_store_str(&pos, &nop->old_name, old_name);

	_count_node_ops[type]++;
	dm_list_add(&_node_ops, &nop->list);

	_log_node_op("Stacking", nop);

	return 1;
}

int add_dev_node(const char *dev_name, uint32_t major, uint32_t minor,
		 uid_t uid, gid_t gid, mode_t mode, int check_udev, unsigned rely_on_udev)
{
	return _stack_node_op(NODE_ADD, dev_name, major, minor, uid,
			      gid, mode, "", 0, 0, check_udev, rely_on_udev);
}

int rename_dev_node(const char *old_name, const char *new_name, int check_udev, unsigned rely_on_udev)
{
	return _stack_node_op(NODE_RENAME, new_name, 0, 0, 0,
			      0, 0, old_name, 0, 0, check_udev, rely_on_udev);
}

int rm_dev_node(const char *dev_name, int check_udev, unsigned rely_on_udev)
{
	return _stack_node_op(NODE_DEL, dev_name, 0, 0, 0,
			      0, 0, "", 0, 0, check_udev, rely_on_udev);
}

int set_dev_node_read_ahead(const char *dev_name,
                            uint32_t major, uint32_t minor,
			    uint32_t read_ahead, uint32_t read_ahead_flags)
{
	if (read_ahead == DM_READ_AHEAD_AUTO)
		return 1;

	return _stack_node_op(NODE_READ_AHEAD, dev_name, major, minor, 0, 0,
                              0, "", read_ahead, read_ahead_flags, 0, 0);
}

void update_devs(void)
{
}

const char *dm_dir(void)
{
	return _dm_dir;
}

const char *dm_sysfs_dir(void)
{
	return _sysfs_dir;
}

int dm_mknodes(const char *name)
{
	struct dm_task *dmt;
	int r = 0;

	if (!(dmt = dm_task_create(DM_DEVICE_MKNODES)))
		return 0;

	if (name && !dm_task_set_name(dmt, name))
		goto out;

	if (!dm_task_no_open_count(dmt))
		goto out;

	r = dm_task_run(dmt);

out:
	dm_task_destroy(dmt);
	return r;
}

int dm_driver_version(char *version, size_t size)
{
	struct dm_task *dmt;
	int r = 0;

	if (!(dmt = dm_task_create(DM_DEVICE_VERSION)))
		return 0;

	if (!dm_task_run(dmt))
		syslog(LOG_INFO, "Failed to get driver version");

	if (!dm_task_get_driver_version(dmt, version, size))
		goto out;

	r = 1;

out:
	dm_task_destroy(dmt);
	return r;
}

int dm_task_set_cookie(struct dm_task *dmt, uint32_t *cookie, uint16_t flags)
{
	if (dm_cookie_supported())
		dmt->event_nr = flags << DM_UDEV_FLAGS_SHIFT;
	*cookie = 0;

	return 1;
}
