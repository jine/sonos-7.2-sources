/*
 *   fs/cifs/cifs_unicode.c
 *
 *   Copyright (c) International Business Machines  Corp., 2000,2002
 *   Modified by Steve French (sfrench@us.ibm.com)
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or 
 *   (at your option) any later version.
 * 
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software 
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/fs.h>
#include <linux/errno.h>
#include "cifs_unicode.h"
#include "cifs_uniupr.h"
#include "cifspdu.h"
#include "cifs_debug.h"

/*
 * NAME:	cifs_strfromUCS()
 *
 * FUNCTION:	Convert little-endian unicode string to character string
 *
 */
int
cifs_strfromUCS_le(char *to, const wchar_t * from,	/* LITTLE ENDIAN */
		   int len, const struct nls_table *codepage)
{
	int i;
	int outlen = 0;

	for (i = 0; (i < len) && from[i]; i++) {
		int charlen;
		/* 2.4.0 kernel or greater */
		charlen =
		    codepage->uni2char(le16_to_cpu(from[i]), &to[outlen],
				       NLS_MAX_CHARSET_SIZE);
		if (charlen > 0) {
			outlen += charlen;
		} else {
			to[outlen++] = '?';
		}
	}
	to[outlen] = 0;
	return outlen;
}

/*
 * NAME:	cifs_strtoUCS()
 *
 * FUNCTION:	Convert character string to unicode string
 *
 */
int
cifs_strtoUCS(wchar_t * to, const char *from, int len,
	      const struct nls_table *codepage)
{
	int charlen;
	int i;

	for (i = 0; len && *from; i++, from += charlen, len -= charlen) {

		/* works for 2.4.0 kernel or later */
		charlen = codepage->char2uni(from, len, &to[i]);
		if (charlen < 1) {
			cERROR(1,
			       ("cifs_strtoUCS: char2uni returned %d",
				charlen));
			to[i] = cpu_to_le16(0x003f);	/* a question mark */
			charlen = 1;
		}
		to[i] = cpu_to_le16(to[i]);

	}

	to[i] = 0;
	return i;
}

/* set in init (cifsfs.c) */
struct nls_table *cp437_page = NULL;

int
cifs_convertn(char *to, const char *from, int tolimit, int fromlimit,
              const struct nls_table *local_codepage, int direction) 
{
    const struct nls_table *f = local_codepage, *t;
    int size = 0, nc = 0;
    wchar_t c;
    const char* firstchar = from;

    if (!cp437_page &&
        !(cp437_page = load_nls("cp437"))) {
        return nc;
    }
    
    t = cp437_page;
    if (direction == CIFS_TO_LOCAL) {
        f = t;
        t = local_codepage;
    }

    while (((from - firstchar) < (size_t)fromlimit) && *from) {
        /* found #define for "infinity" in this context...(nls.h) */
        size = f->char2uni(from, NLS_MAX_CHARSET_SIZE, &c);
        if (size < 0) {
            /* skip */
            from++;
            continue;
        }
        from += size;
        size = t->uni2char(c, to, tolimit - nc);
        if (size < 0)  {
            /* EINVAL <=> c unmapped in t.  never returned if t = utf8, 
               but still... */
            if (size == -EINVAL) {
                continue;
            }

            /* we null terminate unless we hit the limit exactly (so if we have
             * a character with a two byte encoding, and one byte left in our
             * buffer, we null that out).   */
            if (tolimit != nc) 
                *to = 0;
            return nc;
        }
        to += size;
        nc += size;
    }
    *to = 0;
    return nc;
}

int cifs_convert(char *to, const char *from, int tolimit, 
                 const struct nls_table *local_codepage, int dir)
{
    return cifs_convertn(to, from, tolimit, INT_MAX, local_codepage, dir);
}

/* implicitly assumed here is the fact that one unicode character = one CP
 * 437 char, thus, this function only makes sense in the conversion to local
 * (utf8), as only in that direction do we need to worry about expansion... */
int
cifs_strnlen(const char *s, int lim, const struct nls_table *local) {
    char buf[NLS_MAX_CHARSET_SIZE];
    wchar_t c;
    int nc = 0; 
    int size;

    if (!cp437_page &&
        !(cp437_page = load_nls("cp437"))) {
        return nc;
    }

    while (*s) { 
        size = cp437_page->char2uni(s, NLS_MAX_CHARSET_SIZE, &c);
        if (size < 0) {
            /* skip */
            s++;
            continue;
        }
        s += size;
        size = local->uni2char(c, buf, NLS_MAX_CHARSET_SIZE);
        if (size < 0) {
            /* skip here too */
            continue;
        }
        nc += size;
        if (nc >= lim) {
            return lim;
        }
    }
    return nc;
}
