/*
 * Copyright (c) 2016, Sonos, Inc.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <errno.h>
#include <libgen.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/stat.h>

#include "event_reporter.h"

extern size_t strnlen(const char* str, size_t sz);

static int add_str(struct event_context_t *evctx, const char *str)
{
    size_t len = strlen(str) + 1;
    if (len > (sizeof(evctx->buf) - evctx->bufpos)) {
	return -1;
    }
    strcpy(evctx->buf + evctx->bufpos, str);
    evctx->bufpos += len;
    return 0;
}

static int add_int(struct event_context_t *evctx, int val)
{
    char buf[15];
    snprintf(buf, sizeof(buf), "%d", val);
    return add_str(evctx, buf);
}

static int add_uint(struct event_context_t *evctx, unsigned int val)
{
    char buf[15];
    snprintf(buf, sizeof(buf), "%u", val);
    return add_str(evctx, buf);
}

static unsigned char is_ok_to_write(const char* filename, size_t num_bytes)
{
    struct stat s;
    int ret = stat(filename, &s);
    if (ret && errno == ENOENT) {
        return 1;
    } else if (ret) {
        return 0;
    }
    return num_bytes + s.st_size <= TOTAL_SIZE_LIMIT;
}

static void comm_name(char buf[])
{
    char tmpbuf[32]; // would need to increase this for a long directory name

    memset(tmpbuf, '\0', sizeof(tmpbuf));

    int fd = open("/proc/self/cmdline", O_RDONLY);
    if (fd < 0) {
        strcpy(buf, "unknown");
        goto fdend;
    }

    int amt = read(fd, tmpbuf, sizeof(tmpbuf) - 1);

    if (amt <= 0) {
        strcpy(buf, "unknown");
        goto end;
    }

    char *procname = basename(tmpbuf);
    strncpy(buf, procname, TASK_COMM_LEN);
    buf[TASK_COMM_LEN] = '\0';

end:
    close(fd);
fdend:
    return;
}

int init_event_ctx(struct event_context_t *evctx, const char* event_name)
{
    memset(evctx, 0, sizeof(struct event_context_t));

    size_t len = strnlen(event_name, sizeof(evctx->event_name));

    if (len >= sizeof(evctx->event_name)) {
	return -1;
    }

    strcpy(evctx->event_name, event_name);

    return add_str(evctx, event_name) || add_int(evctx, time(0));
}

int add_pair_int(struct event_context_t *evctx, const char* key, int val)
{
    return add_str(evctx, key) || add_int(evctx, val);
}

int add_pair_uint(struct event_context_t *evctx, const char* key, unsigned int val)
{
    return add_str(evctx, key) || add_uint(evctx, val);
}

int add_pair_str(struct event_context_t *evctx, const char *key, const char *val)
{
    return add_str(evctx, key) || add_str(evctx, val);
}

int report_event(struct event_context_t *evctx)
{
    int ret = -1;

    /* Terminate record with a newline */
    if (evctx->bufpos >= MAX_EVENT_SIZE) {
        goto fdend;
    }
    evctx->buf[evctx->bufpos++] = '\n';

    char filename[MAX_PATHNAME_SIZE];
    /* compute the path name */
    {
        const char *format = "%s/%s";
        char procname[TASK_COMM_LEN + 1];
        comm_name(procname);
        int pathsize = snprintf(filename, MAX_PATHNAME_SIZE, format, EXTERNAL_EVENT_DIR, procname);
        if (pathsize < 0 || (unsigned int)pathsize >= MAX_PATHNAME_SIZE) {
            goto fdend;
        }
    }

    /* create directory if needed */
    {
        int res = mkdir(EXTERNAL_EVENT_DIR, 0777);
        if (res && errno != EEXIST) {
            goto fdend;
        }
    }

    /* check for size constraints */
    if (!is_ok_to_write(filename, evctx->bufpos)) {
        goto fdend;
    }

    int fd = open(filename, O_CREAT | O_WRONLY | O_APPEND, 0777);

    if (fd < 0) {
	goto fdend;
    }

    if (flock(fd, LOCK_EX | LOCK_NB)) {
        // Possibly couldn't get the lock. Fail anyway.
        goto end;
    }

    size_t amt_written = 0;
    while (amt_written < evctx->bufpos) {
	int amt = write(fd, evctx->buf + amt_written, evctx->bufpos - amt_written);
	if (amt > 0) {
	    amt_written += amt;
	} else {
            goto end;
	}
    }
    ret = 0;

end:
    close(fd);
fdend:
    return ret;
}
