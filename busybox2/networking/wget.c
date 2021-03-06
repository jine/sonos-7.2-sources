/* vi: set sw=4 ts=4: */
/*
 * wget - retrieve a file using HTTP or FTP
 *
 * Chip Rosenthal Covad Communications <chip@laserlink.net>
 *
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/ioctl.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <getopt.h>

#include "busybox.h"

#define WGET_EXIT_TIMEOUT  2

struct host_info {
	char *host;
	int port;
	char *path;
	int is_ftp;
	char *user;
};

static void parse_url(char *url, struct host_info *h);
static FILE *open_socket(struct sockaddr_in *s_in, int port);
static char *gethdr(char *buf, size_t bufsiz, FILE *fp, int *istrunc);
static int ftpcmd(char *s1, char *s2, FILE *fp, char *buf);

/* Globals (can be accessed from signal handlers */
static off_t filesize = 0;		/* content-length of the file */
static int chunked = 0;			/* chunked transfer encoding */
#ifdef CONFIG_FEATURE_WGET_STATUSBAR
static char *curfile;			/* Name of current file being transferred. */
static unsigned int start;	    /* Time a transfer started. */
/* For progressmeter() -- number of seconds before xfer considered "stalled" */
static const int STALLTIME = 5;
#endif
static void progressmeter(int flag);
static volatile unsigned long statbytes = 0; /* Number of bytes transferred so far. */

static void timeout_monitor(int flag);
static unsigned int timeout_sec = 0;
		
static void close_and_delete_outfile(FILE* output, char *fname_out, int do_continue)
{
	if (output != stdout && do_continue==0) {
		fclose(output);
		unlink(fname_out);
	}
}

/* Read NMEMB elements of SIZE bytes into PTR from STREAM.  Returns the
 * number of elements read, and a short count if an eof or non-interrupt
 * error is encountered.  */
static size_t safe_fread(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
	size_t ret = 0;

	do {
		clearerr(stream);
		ret += fread((char *)ptr + (ret * size), size, nmemb - ret, stream);
	} while (ret < nmemb && ferror(stream) && errno == EINTR);

	return ret;
}

/* Write NMEMB elements of SIZE bytes from PTR to STREAM.  Returns the
 * number of elements written, and a short count if an eof or non-interrupt
 * error is encountered.  */
static size_t safe_fwrite(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
	size_t ret = 0;

	do {
		clearerr(stream);
		ret += fwrite((char *)ptr + (ret * size), size, nmemb - ret, stream);
	} while (ret < nmemb && ferror(stream) && errno == EINTR);

	return ret;
}

/* Read a line or SIZE - 1 bytes into S, whichever is less, from STREAM.
 * Returns S, or NULL if an eof or non-interrupt error is encountered.  */
static char *safe_fgets(char *s, int size, FILE *stream)
{
	char *ret;

	do {
		clearerr(stream);
		ret = fgets(s, size, stream);
	} while (ret == NULL && ferror(stream) && errno == EINTR);

	return ret;
}

#define close_delete_and_die(s...) { \
	close_and_delete_outfile(output, fname_out, do_continue); \
	bb_error_msg_and_die(s); }


#ifdef CONFIG_FEATURE_WGET_AUTHENTICATION
/*
 *  Base64-encode character string
 *  oops... isn't something similar in uuencode.c?
 *  It would be better to use already existing code
 */
char *base64enc(char *p, char *buf, int len) {

        char al[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
                    "0123456789+/";
		char *s = buf;

        while(*p) {
				if (s >= buf+len-4)
					bb_error_msg_and_die("buffer overflow");
                *(s++) = al[(*p >> 2) & 0x3F];
                *(s++) = al[((*p << 4) & 0x30) | ((*(p+1) >> 4) & 0x0F)];
                *s = *(s+1) = '=';
                *(s+2) = 0;
                if (! *(++p)) break;
                *(s++) = al[((*p << 2) & 0x3C) | ((*(p+1) >> 6) & 0x03)];
                if (! *(++p)) break;
                *(s++) = al[*(p++) & 0x3F];
        }

		return buf;
}
#endif

int wget_main(int argc, char **argv)
{
	int n, try=5, status;
	int port;
	char *proxy = 0;
	char *dir_prefix=NULL;
	char *s, buf[512];
	struct stat sbuf;
	char extra_headers[1024];
	char *extra_headers_ptr = extra_headers;
	int extra_headers_left = sizeof(extra_headers);
	int which_long_opt = 0, option_index = -1;
	struct host_info server, target;
	struct sockaddr_in s_in;

	FILE *sfp = NULL;			/* socket to web/ftp server			*/
	FILE *dfp = NULL;			/* socket to ftp server (data)		*/
	char *fname_out = NULL;		/* where to direct output (-O)		*/
	int do_continue = 0;		/* continue a prev transfer (-c)	*/
	long beg_range = 0L;		/*   range at which continue begins	*/
	int got_clen = 0;			/* got content-length: from server	*/
	FILE *output;				/* socket to web server				*/
	int quiet_flag = FALSE;		/* Be verry, verry quiet...			*/
	int noproxy = 0;            /* Use proxies if env vars are set  */
	FILE *report_fp = NULL;		/* report file handle				*/
	const char *report_fname = NULL;  /* where to write report 		*/
	const char *timeout = NULL; /* read timeout in secs				*/

#define LONG_HEADER    1
#define LONG_PASSIVE   2

// -C option is non-standard. Writes report containing various
// parameters for the download (server IP address, content-length, etc) to the
// specified file prior to outputing the first bytes of content.
// 
// -T option is standard and implemented in more recent versions of
//  busybox using non-blocking sockets. The implementation here uses an
//  alarm signal handler to monitor transfer progress.

	struct option long_options[] = {
		{ "continue",        0, NULL, 'c' },
		{ "quiet",           0, NULL, 'q' },
		{ "output-document", 1, NULL, 'O' },
		{ "header",          1, &which_long_opt, LONG_HEADER },
		{ "proxy",           1, NULL, 'Y' },
		{ "passive-ftp",     0, &which_long_opt, LONG_PASSIVE },
		{ "timeout",         1, NULL, 'T' },
		{ "report",          1, NULL, 'C' },
		{ 0,                 0, 0, 0 }
	};
	/*
	 * Crack command line.
	 */
	while ((n = getopt_long(argc, argv, "cqO:P:Y:T:C:", long_options, &option_index)) != EOF) {
		switch (n) {
		case 'c':
			++do_continue;
			break;
		case 'P':
			dir_prefix = optarg;
			break;
		case 'C':
			report_fname = optarg;
			break;
		case 'T':
			timeout = optarg;
			break;
		case 'q':
			quiet_flag = TRUE;
			break;
		case 'O':
			/* can't set fname_out to NULL if outputting to stdout, because
			 * this gets interpreted as the auto-gen output filename
			 * case below  - tausq@debian.org
			 */
			fname_out = optarg;
			break;
		case 'Y':
			if (strcmp(optarg, "off") == 0)
				noproxy=1;	
			break;
		case 0:
			switch (which_long_opt) {
				case LONG_HEADER: {
					int arglen = strlen(optarg);
					if(extra_headers_left - arglen - 2 <= 0)
						bb_error_msg_and_die("extra_headers buffer too small(need %i)", extra_headers_left - arglen);
					strcpy(extra_headers_ptr, optarg);
					extra_headers_ptr += arglen;
					extra_headers_left -= ( arglen + 2 );
					*extra_headers_ptr++ = '\r';
					*extra_headers_ptr++ = '\n';
					*(extra_headers_ptr + 1) = 0;
					break;
				}
				case LONG_PASSIVE:
					// ignore -- we always use passive mode
					break;
			}
			break;
		default:
			bb_show_usage();
		}
	}

	if (argc - optind != 1)
			bb_show_usage();

	parse_url(argv[optind], &target);
	server.host = target.host;
	server.port = target.port;

	if (timeout) {
		timeout_sec = strtoul(timeout, NULL, 10);
	}

	/*
	 * Use the proxy if necessary.
	 */
	if (!noproxy) {
		proxy = getenv(target.is_ftp ? "ftp_proxy" : "http_proxy");
		if (proxy)
			parse_url(bb_xstrdup(proxy), &server);
	}
	
	/* Guess an output filename */
	if (!fname_out) {
		fname_out = 
#ifdef CONFIG_FEATURE_WGET_STATUSBAR
			curfile = 
#endif
			bb_get_last_path_component(target.path);
		if (fname_out==NULL || strlen(fname_out)<1) {
			fname_out = 
#ifdef CONFIG_FEATURE_WGET_STATUSBAR
				curfile = 
#endif
				"index.html";
		}
		if (dir_prefix != NULL)
			fname_out = concat_path_file(dir_prefix, fname_out);
#ifdef CONFIG_FEATURE_WGET_STATUSBAR
	} else {
		curfile = bb_get_last_path_component(fname_out);
#endif
	}
	if (do_continue && !fname_out)
		bb_error_msg_and_die("cannot specify continue (-c) without a filename (-O)");


	/*
	 * Open the output file stream.
	 */
	if (strcmp(fname_out, "-") == 0) {
		output = stdout;
		quiet_flag = TRUE;
	} else {
		output = bb_xfopen(fname_out, (do_continue ? "a" : "w"));
	}

	/*
	 * Determine where to start transfer.
	 */
	if (do_continue) {
		if (fstat(fileno(output), &sbuf) < 0)
			bb_perror_msg_and_die("fstat()");
		if (sbuf.st_size > 0)
			beg_range = sbuf.st_size;
		else
			do_continue = 0;
	}

	if (report_fname) {
		report_fp = bb_xfopen(report_fname, "w");
		/* Success or die */
		fprintf(report_fp, "Server: %s\n", server.host);
		fflush(report_fp);
	}

	/* We want to do exactly _one_ DNS lookup, since some
	 * sites (i.e. ftp.us.debian.org) use round-robin DNS
	 * and we want to connect to only one IP... */
	bb_lookup_host(&s_in, server.host, NULL);
	if (quiet_flag==FALSE) {
		fprintf(stdout, "Connecting to %s[%s]:%d\n",
				server.host, inet_ntoa(s_in.sin_addr), server.port);
	}
	if (report_fp) {
		fprintf(report_fp, "ServerIP: %s\n", inet_ntoa(s_in.sin_addr));
		fflush(report_fp);
	}

	if (proxy || !target.is_ftp) {
		/*
		 *  HTTP session
		 */
		do {
			got_clen = chunked = 0;

			if (! --try)
				close_delete_and_die("too many redirections");

			/*
			 * Open socket to http server
			 */
			if (sfp) fclose(sfp);
			sfp = open_socket(&s_in, server.port);
			
			/*
			 * Send HTTP request.
			 */
			if (proxy) {
				const char *format = "GET %stp://%s:%d/%s HTTP/1.1\r\n";
#ifdef CONFIG_FEATURE_WGET_IP6_LITERAL
				if (strchr (target.host, ':'))
					format = "GET %stp://[%s]:%d/%s HTTP/1.1\r\n";
#endif
				fprintf(sfp, format,
					target.is_ftp ? "f" : "ht", target.host,
					target.port, target.path);
			} else {
				fprintf(sfp, "GET /%s HTTP/1.1\r\n", target.path);
			}

			fprintf(sfp, "Host: %s\r\nUser-Agent: Wget\r\n", target.host);

#ifdef CONFIG_FEATURE_WGET_AUTHENTICATION
			if (target.user) {
				fprintf(sfp, "Authorization: Basic %s\r\n",
					base64enc(target.user, buf, sizeof(buf)));
			}
			if (proxy && server.user) {
				fprintf(sfp, "Proxy-Authorization: Basic %s\r\n",
					base64enc(server.user, buf, sizeof(buf)));
			}
#endif

			if (do_continue)
				fprintf(sfp, "Range: bytes=%ld-\r\n", beg_range);
			if(extra_headers_left < sizeof(extra_headers))
				fputs(extra_headers,sfp);
			fprintf(sfp,"Connection: close\r\n\r\n");

			/*
		 	* Retrieve HTTP response line and check for "200" status code.
		 	*/
read_response:
			if (fgets(buf, sizeof(buf), sfp) == NULL)
				close_delete_and_die("no response from server");
				
			for (s = buf ; *s != '\0' && !isspace(*s) ; ++s)
			;
			for ( ; isspace(*s) ; ++s)
			;
			switch (status = atoi(s)) {
				case 0:
				case 100:
					while (gethdr(buf, sizeof(buf), sfp, &n) != NULL);
					goto read_response;
				case 200:
					if (do_continue && output != stdout)
						output = freopen(fname_out, "w", output);
					do_continue = 0;
					break;
				case 300:	/* redirection */
				case 301:
				case 302:
				case 303:
					break;
				case 206:
					if (do_continue)
						break;
					/*FALLTHRU*/
				default:
					chomp(buf);
					close_delete_and_die("server returned error %d: %s", atoi(s), buf);
			}
		
			/*
			 * Retrieve HTTP headers.
			 */
			while ((s = gethdr(buf, sizeof(buf), sfp, &n)) != NULL) {
				if (strcasecmp(buf, "content-length") == 0) {
					filesize = atol(s);
					got_clen = 1;
					continue;
				}
				if (strcasecmp(buf, "transfer-encoding") == 0) {
					if (strcasecmp(s, "chunked") == 0) {
						chunked = got_clen = 1;
					} else {
					close_delete_and_die("server wants to do %s transfer encoding", s);
					}
				}
				if (strcasecmp(buf, "location") == 0) {
					if (s[0] == '/')
						target.path = bb_xstrdup(s+1);
					else {
						parse_url(bb_xstrdup(s), &target);
						if (!proxy) {
							server.host = target.host;
							server.port = target.port;
							if (report_fp) {
								fprintf(report_fp, "Server: %s\n", server.host);
								fflush(report_fp);
							}
						}
						bb_lookup_host(&s_in, server.host, NULL);
						if (report_fp) {
							fprintf(report_fp, "ServerIP: %s\n", inet_ntoa(s_in.sin_addr));
							fflush(report_fp);
						}
						break;
					}
				}
			}
		} while(status >= 300);
		
		dfp = sfp;
	}
	else
	{
		/*
		 *  FTP session
		 */
		if (! target.user)
			target.user = bb_xstrdup("anonymous:busybox@");

		sfp = open_socket(&s_in, server.port);
		if (ftpcmd(NULL, NULL, sfp, buf) != 220)
			close_delete_and_die("%s", buf+4);

		/* 
		 * Splitting username:password pair,
		 * trying to log in
		 */
		s = strchr(target.user, ':');
		if (s)
			*(s++) = '\0';
		switch(ftpcmd("USER ", target.user, sfp, buf)) {
			case 230:
				break;
			case 331:
				if (ftpcmd("PASS ", s, sfp, buf) == 230)
					break;
				/* FALLTHRU (failed login) */
			default:
				close_delete_and_die("ftp login: %s", buf+4);
		}
		
		ftpcmd("CDUP", NULL, sfp, buf);
		ftpcmd("TYPE I", NULL, sfp, buf);
		
		/*
		 * Querying file size
		 */
		if (ftpcmd("SIZE /", target.path, sfp, buf) == 213) {
			filesize = atol(buf+4);
			got_clen = 1;
		}
		
		/*
		 * Entering passive mode
		 */
		if (ftpcmd("PASV", NULL, sfp, buf) !=  227)
			close_delete_and_die("PASV: %s", buf+4);
		s = strrchr(buf, ',');
		*s = 0;
		port = atoi(s+1);
		s = strrchr(buf, ',');
		port += atoi(s+1) * 256;
		dfp = open_socket(&s_in, port);

		if (do_continue) {
			sprintf(buf, "REST %ld", beg_range);
			if (ftpcmd(buf, NULL, sfp, buf) != 350) {
				if (output != stdout)
					output = freopen(fname_out, "w", output);
				do_continue = 0;
			} else
				filesize -= beg_range;
		}
		
		if (ftpcmd("RETR /", target.path, sfp, buf) > 150)
			close_delete_and_die("RETR: %s", buf+4);

	}


	/*
	 * Retrieve file
	 */

	if (report_fp && filesize > 0) {
		fprintf(report_fp, "Content-length: %lu\n", (unsigned long)filesize);
		fflush(report_fp);
	}

	if (chunked) {
		fgets(buf, sizeof(buf), dfp);
		filesize = strtol(buf, (char **) NULL, 16);
	}

	if (quiet_flag==FALSE) {
		progressmeter(-1);
	} else if (timeout_sec) {
		// If the progress meter is running we will do timeout checking there.
		// Otherwise we need a dedicated monitor.
		timeout_monitor(-1);
	}

	do {
		while ((filesize > 0 || !got_clen) && (n = safe_fread(buf, 1, ((chunked || got_clen) && (filesize < sizeof(buf)) ? filesize : sizeof(buf)), dfp)) > 0) {
			if (safe_fwrite(buf, 1, n, output) != n) {
				bb_perror_msg_and_die("write error");
			}
			statbytes+=n;
			if (got_clen) {
				filesize -= n;
			}
		}

		if (chunked) {
			safe_fgets(buf, sizeof(buf), dfp); /* This is a newline */
			safe_fgets(buf, sizeof(buf), dfp);
			filesize = strtol(buf, (char **) NULL, 16);
			if (filesize==0) {
				chunked = 0; /* all done! */
			}
		}

		if (n == 0 && ferror(dfp)) {
			bb_perror_msg_and_die("network read error");
		}
	} while (chunked);

	if (quiet_flag==FALSE) {
		progressmeter(0);
	} else if (timeout_sec) {
		timeout_monitor(0);
	}

	if (report_fp) {
		fclose(report_fp);
	}

	if (!proxy && target.is_ftp) {
		fclose(dfp);
		if (ftpcmd(NULL, NULL, sfp, buf) != 226)
			bb_error_msg_and_die("ftp error: %s", buf+4);
		ftpcmd("QUIT", NULL, sfp, buf);
	}
	exit(EXIT_SUCCESS);
}


void parse_url(char *url, struct host_info *h)
{
	char *cp, *sp, *up, *pp;

	if (strncmp(url, "http://", 7) == 0) {
		h->port = 80;
		h->host = url + 7;
		h->is_ftp = 0;
	} else if (strncmp(url, "ftp://", 6) == 0) {
		h->port = 21;
		h->host = url + 6;
		h->is_ftp = 1;
	} else
		bb_error_msg_and_die("not an http or ftp url: %s", url);

	sp = strchr(h->host, '/');
	if (sp != NULL) {
		*sp++ = '\0';
		h->path = sp;
	} else
		h->path = bb_xstrdup("");

	up = strrchr(h->host, '@');
	if (up != NULL) {
		h->user = h->host;
		*up++ = '\0';
		h->host = up;
	} else
		h->user = NULL;

	pp = h->host;

#ifdef CONFIG_FEATURE_WGET_IP6_LITERAL
	if (h->host[0] == '[') {
		char *ep;

		ep = h->host + 1;
		while (*ep == ':' || isxdigit (*ep))
			ep++;
		if (*ep == ']') {
			h->host++;
			*ep = '\0';
			pp = ep + 1;
		}
	}
#endif

	cp = strchr(pp, ':');
	if (cp != NULL) {
		*cp++ = '\0';
		h->port = atoi(cp);
	}

}


FILE *open_socket(struct sockaddr_in *s_in, int port)
{
	int fd;
	FILE *fp;
	int one=1;

	if (port>0 && port < 65536) {
		s_in->sin_port=htons(port);
	}

	fd=xconnect(s_in);

    setsockopt(fd,SOL_SOCKET,SO_KEEPALIVE,&one,sizeof(one));
	/*
	 * Get the server onto a stdio stream.
	 */
	if ((fp = fdopen(fd, "r+")) == NULL)
		bb_perror_msg_and_die("fdopen()");

	return fp;
}


char *gethdr(char *buf, size_t bufsiz, FILE *fp, int *istrunc)
{
	char *s, *hdrval;
	int c;

	*istrunc = 0;

	/* retrieve header line */
	if (fgets(buf, bufsiz, fp) == NULL)
		return NULL;

	/* see if we are at the end of the headers */
	for (s = buf ; *s == '\r' ; ++s)
		;
	if (s[0] == '\n')
		return NULL;

	/* convert the header name to lower case */
	for (s = buf ; isalnum(*s) || *s == '-' ; ++s)
		*s = tolower(*s);

	/* verify we are at the end of the header name */
	if (*s != ':')
		bb_error_msg_and_die("bad header line: %s", buf);

	/* locate the start of the header value */
	for (*s++ = '\0' ; *s == ' ' || *s == '\t' ; ++s)
		;
	hdrval = s;

	/* locate the end of header */
	while (*s != '\0' && *s != '\r' && *s != '\n')
		++s;

	/* end of header found */
	if (*s != '\0') {
		*s = '\0';
		return hdrval;
	}

	/* Rats!  The buffer isn't big enough to hold the entire header value. */
	while (c = getc(fp), c != EOF && c != '\n')
		;
	*istrunc = 1;
	return hdrval;
}

static int ftpcmd(char *s1, char *s2, FILE *fp, char *buf)
{
	char *p;
	
	if (s1) {
		if (!s2) s2="";
		fprintf(fp, "%s%s\r\n", s1, s2);
		fflush(fp);
	}
	
	do {
		p = fgets(buf, 510, fp);
		if (!p)
			bb_perror_msg_and_die("fgets()");
	} while (! isdigit(buf[0]) || buf[3] != ' ');
	
	return atoi(buf);
}


static unsigned int monotonic_sec(void)
{
	return time(0);
}

static void
alarmtimer(int wait)
{
	struct itimerval itv;

	itv.it_value.tv_sec = wait;
	itv.it_value.tv_usec = 0;
	itv.it_interval = itv.it_value;
	setitimer(ITIMER_REAL, &itv, NULL);
}

#ifdef CONFIG_FEATURE_WGET_STATUSBAR
/* Stuff below is from BSD rcp util.c, as added to openshh. 
 * Original copyright notice is retained at the end of this file.
 * 
 */ 


static int
getttywidth(void)
{
	int width=0;
	get_terminal_width_height(0, &width, NULL);
	return (width);
}

static void
progressmeter(int flag)
{
	int save_errno = errno;
	static const char prefixes[] = " KMGTP";
	static unsigned int lastupdate;
	static off_t lastsize, totalsize;
	unsigned int now, wait;
	off_t cursize, abbrevsize;
	double elapsed;
	int ratio, barlength, i, remaining;
	char buf[256];

	now = monotonic_sec();

	if (flag == -1) {
		start = now;
		lastupdate = now;
		lastsize = 0;
		totalsize = filesize; /* as filesize changes.. */
	}

	cursize = statbytes;
	if (totalsize != 0 && !chunked) {
		ratio = 100.0 * cursize / totalsize;
		ratio = MAX(ratio, 0);
		ratio = MIN(ratio, 100);
	} else
		ratio = 100;

	snprintf(buf, sizeof(buf), "\r%-20.20s %3d%% ", curfile, ratio);

	barlength = getttywidth() - 51;
	if (barlength > 0) {
		i = barlength * ratio / 100;
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
			 "|%.*s%*s|", i,
			 "*****************************************************************************"
			 "*****************************************************************************",
			 barlength - i, "");
	}
	i = 0;
	abbrevsize = cursize;
	while (abbrevsize >= 100000 && i < sizeof(prefixes)) {
		i++;
		abbrevsize >>= 10;
	}
	snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " %5d %c%c ",
		 (int) abbrevsize, prefixes[i], prefixes[i] == ' ' ? ' ' :
		 'B');

	wait = now - lastupdate;
	if (cursize > lastsize) {
		lastupdate = now;
		lastsize = cursize;
		if (wait >= STALLTIME) {
			start += wait;
		}
		wait = 0;
	} else if (timeout_sec && wait > timeout_sec) {
		// Timeout. Bye-bye.
		exit(WGET_EXIT_TIMEOUT);
	}
	elapsed = now - start; 

	if (wait >= STALLTIME) {
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
			 " - stalled -");
	} else if (statbytes <= 0 || elapsed <= 0.0 || cursize > totalsize || chunked) {
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
			 "   --:-- ETA");
	} else {
		remaining = (int) (totalsize / (statbytes / elapsed) - elapsed);
		i = remaining / 3600;
		if (i)
			snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
				 "%2d:", i);
		else
			snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
				 "   ");
		i = remaining % 3600;
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
			 "%02d:%02d ETA", i / 60, i % 60);
	}
	write(fileno(stderr), buf, strlen(buf));

	if (flag == -1) {
		struct sigaction sa;
		sa.sa_handler = progressmeter;
		sigemptyset(&sa.sa_mask);
		sa.sa_flags = SA_RESTART;
		sigaction(SIGALRM, &sa, NULL);
		alarmtimer(1);
	} else if (flag == 0) {
		alarmtimer(0);
		statbytes = 0;
		putc('\n', stderr);
	}
	errno = save_errno;
}

#else

static void progressmeter(int flag) 
{
	if (timeout_sec) {
		timeout_monitor(flag);
	}
}

#endif // CONFIG_FEATURE_WGET_STATUSBAR

/* Check for network read time out. We only detect timeouts on downloading the
 * content itself. Most of this logic is distilled from the progress meter
 * above.
 *
 * @param flag -1 on first call to setup alarm handler.
 *  		   non-zero when called by the signal handler.
 *  		   0 on last call to disable.
 */
static void timeout_monitor(int flag)
{
	unsigned int since_last_update;
	unsigned int now;
	static unsigned int lastupdate;
	static off_t lastsize;

	/* We can be called from signal handler */
	int save_errno = errno;

	now = monotonic_sec();

	if (flag == -1) { /* first call */
		lastupdate = now;
		lastsize = 0;
	}

	since_last_update = now - lastupdate;
	if (statbytes > lastsize) {
		lastupdate = now;
		lastsize = statbytes;
	} else if (since_last_update >= timeout_sec) {
		// Timeout. Bye-bye.
		exit(WGET_EXIT_TIMEOUT);
	}

	if (flag == -1) {
		struct sigaction sa;
		sa.sa_handler = timeout_monitor;
		sigemptyset(&sa.sa_mask);
		sa.sa_flags = SA_RESTART;
		sigaction(SIGALRM, &sa, NULL);
		alarmtimer(1);
	} else if (flag == 0) {
		alarmtimer(0);
	}

	errno = save_errno;
}

/* Original copyright notice which applies to the CONFIG_FEATURE_WGET_STATUSBAR stuff,
 * much of which was blatently stolen from openssh.  */
 
/*-
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. <BSD Advertising Clause omitted per the July 22, 1999 licensing change 
 *		ftp://ftp.cs.berkeley.edu/pub/4bsd/README.Impt.License.Change> 
 *
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	$Id: wget.c,v 1.1 2003/11/26 01:04:40 vangool Exp $
 */



/*
Local Variables:
c-file-style: "linux"
c-basic-offset: 4
tab-width: 4
End:
*/
