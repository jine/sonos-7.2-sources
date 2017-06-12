#ifndef _DEBUG_H
#define _DEBUG_H

#include "libbb_udhcp.h"

#include <stdio.h>
#ifdef SYSLOG
#include <syslog_client.h>
#endif


#ifdef SYSLOG
# define LOG(level, str, args...) syslog(level, str, ## args)

# define OPEN_LOG(name) do {        \
    setlogmask(LOG_UPTO(LOG_INFO)); \
    setmaxlogsize(4 * 1024);        \
    openlog(name, 0, LOG_USER); } while (0);
# define CLOSE_LOG() closelog()

#elif defined STDOUTLOG || defined JFFSFILELOG
extern FILE* debug_file;
# define LOG_EMERG	"EMERGENCY!"
# define LOG_ALERT	"ALERT!"
# define LOG_CRIT	"critical!"
# define LOG_WARNING	"warning"
# define LOG_ERR	"error"
# define LOG_INFO	"info"
# define LOG_DEBUG	"debug"
# define LOG(level, str, args...) do { fprintf(debug_file,"%s, ", level); \
				fprintf(debug_file,str, ## args); \
				fprintf(debug_file,"\n");\
                                fflush(debug_file); } while(0)

# if defined STDOUTLOG
#  define OPEN_LOG(name) do {debug_file = stdout;} while(0)
#  define CLOSE_LOG()
# else
#  define OPEN_LOG(name) do {debug_file = fopen("/jffs/"name,"a");} while(0)
#  define CLOSE_LOG() do {fclose(debug_file);} while(0)
# endif

#else

# define LOG(...)
# define OPEN_LOG(name)
# define CLOSE_LOG()

#endif

#ifdef DEBUG
# undef DEBUG
# define DEBUG(level, str, args...) LOG(level, str, ## args)
# define DEBUGGING
#else
# define DEBUG(level, str, args...) do {;} while(0)
#endif

#endif
