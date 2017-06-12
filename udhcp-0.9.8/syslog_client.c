/*
 * Copyright (c) 2011, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <ctype.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <assert.h>
#include "syslog_client.h"

#ifdef DBG_SYSLOG

#define SYSLOG_DEF_PORT     514
#define SYSLOG_CONFIG_FILE  "/jffs/syslogger.conf"
#define SYSLOG_DEF_LOCATION "/opt/log/"
#define SYSLOG_DEF_FILESIZE (32 * 1024)
#define SYSLOG_MIN_FILESIZE (2 * 1024)
#define SYSLOG_TRUNC_FACTOR 2

#define DBG_SYSLOG_PRINTF(...)
//#define DBG_SYSLOG_PRINTF       printf

static void write_log_msg(int fd, const char* msg, int len);

static char syslog_localhost[64];
static char syslog_ident[64];
static char syslog_location[128];

static int syslog_fd = -1;
static int syslog_mask = LOG_UPTO(LOG_DEBUG);
static struct sockaddr_in syslog_addr;
static int syslog_use_file = 0;
static int syslog_max_filesize = SYSLOG_DEF_FILESIZE;

static char* delimit_token(char *line)
{
    int i, j;
    char* next = NULL;

    for (i = 0; line[i] != '\0'; i++) {
        if (line[i] == ':' || isspace(line[i]) || iscntrl(line[i])) {
            break;
        }
    }

    // See if there is a subsequent valid line.
    for (j = i; line[j] != '\0'; j++) {
        if (line[j] == ':' || isspace(line[j]) || iscntrl(line[j])) {
            continue;
        }
        next = &line[j];
        break;
    }

    line[i] = '\0';

    return next;
}

static int get_config(const char* filename)
{
    char entry[128];
    char* token;
    char* next;
    int rc = 0;
    unsigned short port = SYSLOG_DEF_PORT;

    if (syslog_ident[0] == '\0') {
        return -1;
    }

    if (gethostname(syslog_localhost, sizeof(syslog_localhost)) == -1) {
        strcpy(syslog_localhost, "unknown");
    }

    // Default config.
    snprintf(syslog_location, sizeof(syslog_location), "%s%s.log",
            SYSLOG_DEF_LOCATION, syslog_ident);
    syslog_localhost[sizeof(syslog_localhost) - 1] = 0;
    syslog_use_file = 1;

    FILE *fh = fopen(filename, "rt");
    if (fh == NULL) {
        // If no config file, use defaults
    } else {

        entry[0] = '\0';

        // Read in configuration entry
        while (fgets(entry, sizeof(entry), fh)) {
            if (entry[0] == '#') {
                // Ignore comments
                continue;
            }

            if (entry[0] == '$') {
                // Retrieve the hostname to use.
                delimit_token(entry);
                strncpy(syslog_localhost, &entry[1], sizeof(syslog_localhost));
                syslog_localhost[sizeof(syslog_localhost) - 1] = 0;
            }

            // Check for matching process name
            token = entry;
            next = delimit_token(token);
            if (strcmp(syslog_ident, token) != 0) {
                continue;
            }

            // Read logmask
            if (!next) {
                rc = -2;
                break;
            }

            token = next;
            next = delimit_token(token);
            syslog_mask = strtol(token, NULL, 0);

            // Read output location
            if (!next) {
                rc = -2;
                break;
            }
            token = next;
            next = delimit_token(token);

            strncpy(syslog_location, token, sizeof(syslog_location));
            syslog_location[sizeof(syslog_location) - 1] = '\0';

            if (token[0] == '/') {
                // Local log file
                syslog_use_file = 1;

                // Check for optional max file size
                if (next) {
                    int size;

                    token = next;
                    next = delimit_token(token);

                    size = strtol(token, NULL, 0);
                    if (size == 0) {
                        rc = -2;
                        break;
                    }
                    setmaxlogsize(size);
                }

                rc = 0;
                break;

            } else {
                // hostname of syslogd server. Check for optional port
                if (next) {
                    token = next;
                    next = delimit_token(token);

                    port = strtol(token, NULL, 0);
                    if (port == 0) {
                        rc = -2;
                        break;
                    }
                }

                memset(&syslog_addr, 0, sizeof(syslog_addr));
                syslog_addr.sin_family = AF_INET;
                syslog_addr.sin_port = htons(port);

                struct hostent *srv = gethostbyname(syslog_location);

                if (srv == NULL) {
                    DBG_SYSLOG_PRINTF("Error resolving syslog sever name\n");
                    rc = -3;
                    break;
                }

                memcpy(&syslog_addr.sin_addr, srv->h_addr_list[0],
                        sizeof(syslog_addr.sin_addr));
                rc = 0;
            }
            break;
        }
        fclose(fh);
    }

    if (rc == 0) {
        if (syslog_use_file) {
            syslog_fd = open(syslog_location, O_CREAT | O_RDWR, DEFFILEMODE);
	    if (syslog_fd >= 0) {
	        lseek(syslog_fd, 0, SEEK_END);
	    }
        } else {
            syslog_fd = socket(AF_INET, SOCK_DGRAM, 0);
        }

        if (syslog_fd == -1) {
            DBG_SYSLOG_PRINTF("Error opening syslog for %s\n", syslog_location);
            rc = -4;
        } else {
            DBG_SYSLOG_PRINTF("Syslog enabled (logmask 0x%x) to %s\n", syslog_mask, syslog_location);
        }
    } else {
        DBG_SYSLOG_PRINTF("No valid configuration entry for '%s' (%d)\n", syslog_ident, rc);
    }

    return rc;
}

// Prepare for logging based on configuration settings.
void openlog(const char* ident, int option, int facility)
{
    // Ignored (hardcoded currently).
    (void)option;
    (void)facility;

    if (ident == NULL) {
        ident = "";
    }

    strncpy(syslog_ident, ident, sizeof(syslog_ident));
    syslog_ident[sizeof(syslog_ident) - 1] = 0;

    if (syslog_fd != -1) {
        // If called again, make sure we dont leak a handle.
        close(syslog_fd);
        syslog_fd = -1;
    }

    if (get_config(SYSLOG_CONFIG_FILE) < 0) {
        return;
    }

    return;
}

// Release resources associated with logging.
void closelog()
{
    if (syslog_fd != -1) {
        close(syslog_fd);
        syslog_fd = -1;
    }
}

// Specify what log levels to allow.
//
// Use LOG_MASK or LOG_UPTO macros for preparing the mask.
int setlogmask(int mask)
{
    int prev = syslog_mask;

    syslog_mask = mask;

    return prev;
}

void setmaxlogsize(int size)
{
    // Enforce a minimum to avoid truncating continuously.
    if (size < SYSLOG_MIN_FILESIZE * 2) {
        size = SYSLOG_MIN_FILESIZE * 2;
    }
    syslog_max_filesize = size;
}    

// Generates a syslog message and sends it to the syslogd server or logfile.
void syslog(int level, const char* format, ...)
{
    va_list ap;

    va_start(ap, format);
    vsyslog(level, format, ap);
    va_end(ap);
}

void vsyslog(int level, const char* format, va_list ap)
{
    char tmp[1024];
    char timestamp[32];
    int len;
    unsigned char pri;
    time_t now;
    struct tm *localtm;

    if (syslog_fd == -1) {
        return;
    }

    if ((syslog_mask & LOG_MASK(LOG_PRI(level))) == 0) {
        return;
    }

    now = time(NULL);
    localtm = localtime(&now);
    if (!localtm) {
        return;
    }
    len = strftime(timestamp, sizeof(timestamp), "%b %d %H:%M:%S", localtm);
    if (len == 0) {
        return;
    }
    if (timestamp[4] == '0') {
        // syslogd wants single-digit days to be padded with space not 0.
        timestamp[4] = ' ';
    }

    // Facility and program name are hardcoded
    pri = LOG_USER | LOG_PRI(level);

    // Syslog format is:
    // <pri>Mmm dd hh:mm:ss hostname tag:xxxxx
    len = snprintf(tmp, sizeof(tmp), "<%d>%s %s %s:", pri, timestamp,
            syslog_localhost, syslog_ident);

    len += vsnprintf(tmp + len, sizeof(tmp) - len, format, ap);

    if (len >= (int)sizeof(tmp)) {
        // snprintf had to truncate.
        len = sizeof(tmp);
    }

    // Note, tmp is not necessarily null terminated here.
    if (len > 0) {
        if (tmp[len - 1] == '\n') {
            // Strip trailing newline if present.
            len--;
        }
        if (syslog_use_file) {
            write_log_msg(syslog_fd, tmp, len);
        } else {
            sendto(syslog_fd, tmp, len, 0, (struct sockaddr*)&syslog_addr,
                    sizeof(syslog_addr));
        }
    }
}


// Write a log message to file, truncating file first if necessary.
static void write_log_msg(int fd, const char* msg, int len)
{
    off_t pos = lseek(fd, 0, SEEK_CUR);

    if (pos == (off_t)-1) {
        // Unable to access log file.
        return;
    }

    // Do we need to truncate now?
    if (pos >= syslog_max_filesize) {
        int i;

        // Truncate to a fraction of the max size, maintaining a min size.
        off_t truncated_size = syslog_max_filesize / SYSLOG_TRUNC_FACTOR;

        if (truncated_size < SYSLOG_MIN_FILESIZE) {
            truncated_size = SYSLOG_MIN_FILESIZE;
        }

        char* buf = malloc(truncated_size);
        if (!buf) {
            truncated_size = 0;
            goto cleanup;
        }

        // Copy the last truncated_size bytes and then rewrite them to the
        // start of the file.
        if (lseek(fd, -truncated_size, SEEK_END) == (off_t)-1) {
            truncated_size = 0;
            goto cleanup;
        }
        if (read(fd, buf, truncated_size) != truncated_size) {
            truncated_size = 0;
            goto cleanup;
        }

        // Advance to the beginning of the first complete line.
        for (i = 0; i < truncated_size; i++) {
            if (buf[i] == '\n') {
                i++;
                break;
            }
        }

        truncated_size -= i;

        if (truncated_size > 0) {
            if (lseek(fd, 0, SEEK_SET) == (off_t)-1) {
                truncated_size = 0;
                goto cleanup;
            }
            if (write(fd, &buf[i], truncated_size) != truncated_size) {
                truncated_size = 0;
                goto cleanup;
            }
        }
cleanup:
        free(buf);
        if (ftruncate(fd, truncated_size) != 0) {
            return;
        }
    }

    int ret = write(fd, msg, len);
    assert(ret == len);
    ret = write(fd, "\n", 1);
    assert(1 == ret);
    (void)ret; // Keep the compiler happy on an i386 release build
    fsync(fd);
}


#endif // DBG_SYSLOG
