/*
 * Copyright (c) 2014, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * sonos_fw_whitelist.c.inc: implementation of check_sonos_firmware_whitelist
 *
 * !!NOTE!! this file is meant to be pulled in with #include after the
 * including project has defined some macros:
 *      int SFW_GETCPUID(uint8_t *buf, size_t len)
 *      int SFW_GETSERIAL(uint8_t *buf, size_t len)
 *      SFW_PRINT(fmt, fmtArgs...)
 *      SFW_PLVL_INFO
 *      SFW_PLVL_EMERG
 *      SFW_BE32_TO_CPU(x)
 *
 * Note that SWF_GETSERIAL should return true if it can get a known
 * good (cryptographically secure) value for serial; false otherwise.
 * If it fails then any non-empty whitelist causes us to not boot.
 *
 * Trying to compile this as a .o and link it in multiple places doesn't work
 * well in situations like the u-boot build or the kernel build (hence the
 * weird file inclusion tactic used here).
 */

static void
initialPrintHelper(uint8_t type, const uint8_t* id)
{
    const char* typeName = type == SONOS_FWW_TYPE_CPUID ? "cpuid" : "serial";

    if (type == SONOS_FWW_TYPE_CPUID) {
        SFW_PRINT(SFW_PLVL_INFO
                  "checking %s whitelist (my %s is "
                  SONOS_FWW_FMT_CPUID ")...\n",
                  typeName, typeName, SONOS_FWW_FMTARG_CPUID(id));
    }
    else {
        SFW_PRINT(SFW_PLVL_INFO
                  "checking %s whitelist (my %s is "
                  SONOS_FWW_FMT_SERIAL ")...\n",
                  typeName, typeName, SONOS_FWW_FMTARG_SERIAL(id));
    }
}

static void
foreachPrintHelper(uint8_t type, const uint8_t* id)
{
    if (type == SONOS_FWW_TYPE_CPUID) {
        SFW_PRINT(SFW_PLVL_INFO
                  "checking cpuid "SONOS_FWW_FMT_CPUID"\n",
                  SONOS_FWW_FMTARG_CPUID(id));
    }
    else {
        SFW_PRINT(SFW_PLVL_INFO
                  "checking serial "SONOS_FWW_FMT_SERIAL"\n",
                  SONOS_FWW_FMTARG_SERIAL(id));
    }
}

static int
check_sonos_firmware_whitelist(const SonosFirmwareWhitelistHeader* hdr)
{
    uint32_t numEntries = SFW_BE32_TO_CPU(hdr->numEntries);
    uint8_t type = hdr->whitelistType;
    int foundWhitelistEntry = 0;
    const uint8_t* myId = NULL;
    const uint8_t* listId;
    size_t listIdLen;
    const char* typeName = NULL;
    uint32_t i;
    SonosCpuid_t myCpuid;
    SonosSerial_t mySerial;
    int result = 0;

    if (type != SONOS_FWW_TYPE_CPUID && type != SONOS_FWW_TYPE_SERIAL) {
        SFW_PRINT(SFW_PLVL_EMERG
                  "bad Sonos firmware whitelist type: %d\n", (int)type);
        return result;
    }
    if (numEntries > SONOS_FWW_MAX_ENTRIES) {
        SFW_PRINT(SFW_PLVL_EMERG
                  "Firmware whitelist too large (%lu)\n", (unsigned long)numEntries);
        return result;
    }

    if (type == SONOS_FWW_TYPE_CPUID) {
        typeName = "cpuid";
        if (SFW_GETCPUID(myCpuid.value, sizeof myCpuid.value)) {
            myId = myCpuid.value;
        }
        listIdLen = sizeof(SonosCpuid_t);
    }
    else {
        typeName = "serial";
        if (SFW_GETSERIAL(mySerial.value, sizeof mySerial.value)) {
            myId = mySerial.value;
        }
        listIdLen = sizeof(SonosSerial_t);
    }

    if (myId) {
        initialPrintHelper(type, myId);

        if (numEntries != 0) {
            listId = (const uint8_t*)&hdr[1];
            for (i = 0; i < numEntries; i++, listId += listIdLen) {
                /* don't print more then 10 entries (too slow) */
                if (i < 10) {
                    foreachPrintHelper(type, listId);
                }
                else if (i == 10) {
                    SFW_PRINT(SFW_PLVL_INFO
                              "... checking too many to list ...\n");
                }
                if (memcmp(myId, listId, listIdLen) == 0) {
                    foundWhitelistEntry = 1;
                    break;
                }
            }
            if (!foundWhitelistEntry) {
                SFW_PRINT(SFW_PLVL_EMERG
                          "failed %s whitelist check\n", typeName);
                return 0;
            }
        }
    }
    else {
        SFW_PRINT(SFW_PLVL_EMERG
                  "checking %s whitelist against unknown %s...\n",
                  typeName, typeName);

        if (numEntries != 0) {
            SFW_PRINT(SFW_PLVL_EMERG
                      "non-empty (%d entries) Sonos %s whitelist; dying...\n",
                      (int)numEntries, typeName);
            return 0;
        }
    }

    SFW_PRINT(SFW_PLVL_INFO "whitelist check completed\n");
    return 1;
}

