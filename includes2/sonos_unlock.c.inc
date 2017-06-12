/*
 * Copyright (c) 2015-2016, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * sonos_unlock.c.inc: code for checking unlock signatures and cpuid/serial
 *                     binding signaturees as used with i.mx6 secure boot
 *
 * !!NOTE!! this file is meant to be pulled in with #include after the
 * including project has defined some macros:
 *      SU_CPU_TO_BE32(x)
 *      // next two return true on success, false on failure
 *      int SU_GETCPUID(uint8_t *buf, size_t len)
 *      int SU_GETFUSE(const char *name, uint32_t *pValue)
 *          for fuse names: SONOS_FUSE_UNLOCK_CTR_NAME
 *      SU_PRINT(fmt, fmtArgs...)
 *      SU_PLVL_DEBUG
 *      SU_PLVL_ERR
 *
 * Note that SU_GETCPUID returns true if it can get a known good value for
 * cpuid; false otherwise.
 *
 * Trying to compile this as a .o and link it in multiple places doesn't work
 * well in situations like the u-boot build or the kernel build (hence the
 * weird file inclusion tactic used here).
 */

#include "sonos_unlock.h"

/* try to avoid tons of magic numbers all over this file */
#define SU_MDP_SERIAL_LEN       6
#define SU_CPUID_LEN            8

int sonosUnlockVerifyCpuSerialSig(const uint8_t* serial, size_t serialLen,
                                  const uint8_t* sigBuf, size_t sigBufLen,

                                  SonosSignature* sig,
                                  SonosHashCallback h,
                                  SonosRawVerifyCallback v,
                                  SonosKeyLookupCallback l,
                                  const void* lArg,
                                  SonosKeyReleaseCallback r)
{
    static const uint8_t cpuSerialSigMagic[] =
    {
        0xec, 0x31, 0x14, 0xef
    };
    uint8_t cpuid[SU_CPUID_LEN];
    uint8_t msg[sizeof(cpuSerialSigMagic) + SU_MDP_SERIAL_LEN + sizeof(cpuid)];
    uint8_t* p = msg;
    size_t sigLen = sonosSignatureParse(sig, sigBuf, sigBufLen);

    if (serialLen != SU_MDP_SERIAL_LEN) {
        SU_PRINT(SU_PLVL_ERR
                 "cpuid/serial signature invalid serialLen: %d\n",
                 (int)serialLen);
        return 0;
    }

    if (sigLen == 0) {
        SU_PRINT(SU_PLVL_ERR
                 "cpuid/serial signature failed to parse\n");
        return 0;
    }

    if (!SU_GETCPUID(cpuid, sizeof cpuid)) {
        SU_PRINT(SU_PLVL_ERR "cpuid/serial signature get_cpuid failed\n");
        return 0;
    }

    memcpy(p, cpuSerialSigMagic, sizeof cpuSerialSigMagic);
    p += sizeof cpuSerialSigMagic;
    memcpy(p, serial, SU_MDP_SERIAL_LEN);
    p += SU_MDP_SERIAL_LEN;
    memcpy(p, cpuid, sizeof cpuid);
    p += sizeof cpuid;

    if (!sonosSignatureVerify(sig, h, v, l, lArg, r, msg, sizeof msg,
                              SONOS_CT_INVALID)) {
        SU_PRINT(SU_PLVL_ERR
                 "cpuid/serial signature failed to verify\n");
        return 0;
    }

    SU_PRINT(SU_PLVL_DEBUG
             "cpuid/serial signature verified successfully\n");
    return 1;
}

int sonosUnlockIsDeviceUnlocked(const struct manufacturing_data_page* mdp,
                                const struct manufacturing_data_page3* mdp3,

                                SonosSignature* sig,
                                SonosHashCallback h,
                                SonosRawVerifyCallback v,
                                SonosKeyLookupCallback lookup,
                                const void* lookupArgCpuid,
                                const void* lookupArgUnlock,
                                SonosKeyReleaseCallback r)
{
    static const uint8_t devUnlockSigPrefix[] =
    {
        0x71, 0xde, 0x54, 0xd6,
        0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d,
        0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d, 0x0d,
        0x0d, 0x0d
    };
    uint8_t msg[sizeof(devUnlockSigPrefix) + SU_MDP_SERIAL_LEN +
                2*sizeof(uint32_t)];
    uint8_t* p = msg;
    uint32_t permissions;
    uint32_t fuseval = 0;
    size_t sigLen;

    if (mdp->mdp_version < MDP_VERSION_AUTH_FLAGS ||
        mdp->mdp3_version < MDP3_VERSION_SECURE_BOOT) {
        SU_PRINT(SU_PLVL_INFO "IsDeviceUnlocked versions too low (%d %d)\n",
                 (int)mdp->mdp_version, (int)mdp->mdp3_version);
        return 0;
    }

    sigLen = sonosSignatureParse(sig,
                                 mdp3->mdp3_auth_sig,
                                 sizeof(mdp3->mdp3_auth_sig));
    if (sigLen == 0) {
        SU_PRINT(SU_PLVL_ERR
                 "IsDeviceUnlocked signature failed to parse\n");
        return 0;
    }

    memcpy(p, devUnlockSigPrefix, sizeof devUnlockSigPrefix);
    p += sizeof devUnlockSigPrefix;
    memcpy(p, mdp->mdp_serial, SU_MDP_SERIAL_LEN);
    p += SU_MDP_SERIAL_LEN;
    permissions = SU_CPU_TO_BE32(mdp->mdp_authorized_flags);
    memcpy(p, &permissions, sizeof permissions);
    p += sizeof permissions;
    if (!SU_GETFUSE(SONOS_FUSE_UNLOCK_CTR_NAME, &fuseval)) {
        SU_PRINT(SU_PLVL_ERR
                 "IsDeviceUnlocked unlock counter fuse read failed\n");
        return 0;
    }
    fuseval = SU_CPU_TO_BE32(fuseval);
    memcpy(p, &fuseval, sizeof fuseval);
    p += sizeof fuseval;

    if (sonosSignatureVerify(sig, h, v, lookup, lookupArgUnlock, r,
                             msg, sizeof msg, SONOS_CT_INVALID) != 1) {
        SU_PRINT(SU_PLVL_ERR
                 "IsDeviceUnlocked signature failed to verify\n");
        return 0;
    }

    if (!sonosUnlockVerifyCpuSerialSig(mdp->mdp_serial, SU_MDP_SERIAL_LEN,
                                       mdp3->mdp3_cpuid_sig,
                                       sizeof(mdp3->mdp3_cpuid_sig),
                                       sig, h, v, lookup, lookupArgCpuid, r)) {
        SU_PRINT(SU_PLVL_ERR
                 "IsDeviceUnlocked bad cpu/serial signature\n");
        return 0;
    }

    SU_PRINT(SU_PLVL_DEBUG
             "IsDeviceUnlocked signature verified successfully\n");
    return 1;
}

int sonosUnlockIsAuthFeatureEnabled(uint32_t flag,
                                    const struct manufacturing_data_page* mdp,
                                    const struct manufacturing_data_page3* mdp3,
                                    SonosSignature* sig,
                                    SonosHashCallback h,
                                    SonosRawVerifyCallback v,
                                    SonosKeyLookupCallback lookup,
                                    const void* lookupArgCpuid,
                                    const void* lookupArgUnlock,
                                    SonosKeyReleaseCallback r)
{
#ifdef SONOS_ARCH_ATTR_RELAX_SECBOOT_STRICTNESS
    uint32_t permissions = MDP_AUTH_FLAG_ALL_FEATURES & (~MDP_AUTH_FLAG_MFG_KEY_ENABLE);
    uint32_t features = permissions;
#else
    uint32_t permissions = 0;
    uint32_t features = 0;
#endif
    int enabled;

#if (defined DIAG_BUILD || defined CONFIG_SONOS_DIAGS) && !defined SONOS_STRICT_DIAG_BUILD
    if (flag != MDP_AUTH_FLAG_MFG_KEY_ENABLE) {
        permissions = MDP_AUTH_FLAG_ALL_FEATURES & (~MDP_AUTH_FLAG_MFG_KEY_ENABLE);
        features = permissions;
        goto decision;
    }
#endif

    if (mdp == NULL ||
        mdp->mdp_magic != MDP_MAGIC ||
        mdp->mdp_version < MDP_VERSION_AUTH_FLAGS) {
        goto decision;
    }

    if (!(mdp->mdp_pages_present & MDP_PAGE3_PRESENT) ||
        mdp3 == NULL ||
        mdp3->mdp3_magic != MDP_MAGIC3 ||
        mdp3->mdp3_version < MDP3_VERSION_DEV_CERT) {
        goto decision;
    }

    permissions = 0;
    features = mdp->mdp_sw_features;

    if (!sonosUnlockIsDeviceUnlocked(mdp, mdp3, sig, h, v, lookup,
                                     lookupArgCpuid, lookupArgUnlock, r)) {
        goto decision;
    }
    permissions = mdp->mdp_authorized_flags;

decision:
    enabled = ((permissions & features) & flag) ? 1 : 0;
    SU_PRINT(SU_PLVL_DEBUG
             "IsAuthFeatureEnabled (0x%lx): %s\n",
             (unsigned long)flag, enabled ? "true" : "false");
    return enabled;
}

uint32_t sonosUnlockGetNextFuseVal(uint32_t fuseval)
{
    uint32_t val;
    int bit;

    for (bit = 0, val = 1;
         bit < 32;
         bit++, val <<= 1) {
        if ((val & fuseval) == 0) {
            return val | fuseval;
        }
    }

    return 0;
}

