#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "options.h"

// gcc -m32 -Wall option_test.c -I.. ../i386-debug/options.o

///////////////////////////////////////////////////////////////////////////////
// test simple parsing of padding
int testSimple()
{
    int rc;
    unsigned char *temp;

    char buffer[2048] = {0};
    char options[] = {
        0x00, 0x00, 0x00,
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 
        0xff,
    };
    struct dhcpMessage *msg = (struct dhcpMessage *)&buffer;
    memcpy(msg->options, options, sizeof(options));

    temp = get_option(msg, 0x01);
    if (temp == NULL) {
        printf("%s: expected to find option 0x01\n", __FUNCTION__);
        rc = 1;
    }

    temp = get_option(msg, 0x02);
    if (temp != NULL) {
        printf("%s: expected to not find option 0x02\n", __FUNCTION__);
        rc = 1;
    }

    return rc;
}

///////////////////////////////////////////////////////////////////////////////
// test a payload that has the 'code' at the last byte
int testCodeAtEnd()
{
    int rc;
    unsigned char *temp;

    char buffer[2048] = {0};
    char options[] = {
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 
        0xff,
    };
    struct dhcpMessage *msg = (struct dhcpMessage *)&buffer;
    memcpy(msg->options + 307, options, sizeof(options));

    temp = get_option(msg, 0x01);
    if (temp != NULL) {
        printf("%s: should not find option 0x01\n", __FUNCTION__);
        rc = 1;
    }

    return rc;
}

///////////////////////////////////////////////////////////////////////////////
// test a payload that has the 'code+len' at the last byte
int testCodeLenAtEnd()
{
    int rc;
    unsigned char *temp;

    char buffer[2048] = {0};
    char options[] = {
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 
        0xff,
    };
    struct dhcpMessage *msg = (struct dhcpMessage *)&buffer;
    memcpy(msg->options + 306, options, sizeof(options));

    temp = get_option(msg, 0x01);
    if (temp != NULL) {
        printf("%s: should not find option 0x01\n", __FUNCTION__);
        rc = 1;
    }

    return rc;
}

///////////////////////////////////////////////////////////////////////////////
// test a payload that has the 'code+len+data' at end
int testCodeLenDataAtEnd()
{
    int rc;
    unsigned char *temp;

    char buffer[2048] = {0};
    char options[] = {
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 
        0xff,
    };
    struct dhcpMessage *msg = (struct dhcpMessage *)&buffer;
    memcpy(msg->options + 302, options, sizeof(options));

    temp = get_option(msg, 0x01);
    if (temp == NULL) {
        printf("%s: should find option 0x01\n", __FUNCTION__);
        rc = 1;
    }

    return rc;
}

///////////////////////////////////////////////////////////////////////////////
// test a payload that has an overflow option
int testOverflowOption()
{
    int rc;
    unsigned char *temp;

    char buffer[2048] = {0};
    char file[] = {
        0x01, 0x04, 0x00, 0x00, 0x00, 0x00,
        0xff,
    };
    char sname[] = {
        0x02, 0x04, 0x00, 0x00, 0x00, 0x00,
        0xff,
    };
    char options[] = {
        0x34, 0x01, 0x03, 0x00, 0x00, 0x00,
        0xff,
    };
    struct dhcpMessage *msg = (struct dhcpMessage *)&buffer;
    memcpy(msg->options, options, sizeof(options));
    memcpy(msg->file, file, sizeof(file));
    memcpy(msg->sname, sname, sizeof(sname));

    temp = get_option(msg, 0x34);
    if (temp == NULL) {
        printf("%s: should find overflow option\n", __FUNCTION__);
        rc = 1;
    } else if (*temp != 0x3) {
        printf("%s: overflow option should be 0x3\n", __FUNCTION__);
        rc = 1;
    }

    temp = get_option(msg, 0x01);
    if (temp == NULL) {
        printf("%s: should find option 0x01\n", __FUNCTION__);
        rc = 1;
    }

    temp = get_option(msg, 0x02);
    if (temp == NULL) {
        printf("%s: should find option 0x02\n", __FUNCTION__);
        rc = 1;
    }

    return rc;
}





typedef int (*testFn)();

int main()
{
    int rc = 0;

    testFn testDefs[] = {
        testSimple,
        testCodeAtEnd,
        testCodeLenAtEnd,
        testCodeLenDataAtEnd,
        testOverflowOption,
        NULL
    };
    testFn *tests = testDefs;
    while (*tests != NULL) {
        if ((*tests)()) {
            rc = 1;
        }
        tests++;
    }

    return rc;
}
