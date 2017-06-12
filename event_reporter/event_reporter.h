/*
 * Copyright (c) 2016, Sonos, Inc.
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef EVENT_REPORTER_H
#define EVENT_REPORTER_H

#ifdef SONOS_ARCH_ATTR_SUPPORTS_EXTERNAL_EVENTS

#define TASK_COMM_LEN 16
#define MAX_EVENT_SIZE 128
#define MAX_EVENT_NAME_SIZE 16
#define EXTERNAL_EVENT_DIR "/tmp/event_reporter"
#define MAX_FILENAME_SIZE (TASK_COMM_LEN)
#define MAX_PATHNAME_SIZE (sizeof(EXTERNAL_EVENT_DIR) + 1 + MAX_FILENAME_SIZE)

#if (__SONOS_ZP_RAM_MB__ >= 128)
#define TOTAL_SIZE_LIMIT 1024 * 10
#elif (__SONOS_ZP_RAM_MB__ >= 64)
#define TOTAL_SIZE_LIMIT 1024
#else
#define TOTAL_SIZE_LIMIT 256
#endif

struct event_context_t {
    char buf[MAX_EVENT_SIZE];
    size_t bufpos;
    char event_name[MAX_EVENT_NAME_SIZE];
};

int init_event_ctx(struct event_context_t *evctx, const char* event_name);
int add_pair_int(struct event_context_t *evctx, const char* key, int val);
int add_pair_uint(struct event_context_t *evctx, const char* key, unsigned int val);
int add_pair_str(struct event_context_t *evctx, const char* key, const char *val);
int report_event(struct event_context_t *evctx);

#endif /* SONOS_ARCH_ATTR_SUPPORTS_EXTERNAL_EVENTS */
#endif /* EVENT_REPORTER_H */
