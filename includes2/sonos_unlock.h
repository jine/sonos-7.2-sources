/*
 * Copyright (c) 2015-2016, Sonos, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0
 *
 * sonos_unlock.h:     code for checking unlock signatures and cpuid/serial
 *                     binding signatures as used with i.mx6 secure boot
 */

#ifndef SONOS_UNLOCK_H
#define SONOS_UNLOCK_H

#ifndef SONOS_ARCH_ATTR_SUPPORTS_SECURE_BOOT
#error "this header should only be included on secure boot ARCHes"
#endif

#include "sonos_signature.h"
#include "sonos_signature_common.h"
#include "sonos_signature_verify.h"
#include "mdp.h"

#ifdef __cplusplus
extern "C" {
#endif

struct manufacturing_data_page;
struct manufacturing_data_page3;

#define SONOS_FUSE_GP1_NAME                 "GP1"
#define SONOS_FUSE_GP1_BANK                 4
#define SONOS_FUSE_GP1_WORD                 6

#define SONOS_FUSE_GP2_NAME                 "GP2"
#define SONOS_FUSE_GP2_BANK                 4
#define SONOS_FUSE_GP2_WORD                 7

#if defined(SONOS_ARCH_ATTR_HAS_SECBOOT_IMX6_QUAD)

#define SONOS_FUSE_UNLOCK_CTR_NAME          "MAC0"
#define SONOS_FUSE_UNLOCK_CTR_BANK          4
#define SONOS_FUSE_UNLOCK_CTR_WORD          2

#elif defined(SONOS_ARCH_ATTR_HAS_SECBOOT_IMX6_SOLOX)

#define SONOS_FUSE_UNLOCK_CTR_NAME          "HDCP_KEY33"
#define SONOS_FUSE_UNLOCK_CTR_BANK          10
#define SONOS_FUSE_UNLOCK_CTR_WORD          1

#else
#error "unsupported secure boot SoC"
#endif

int sonosUnlockVerifyCpuSerialSig(const uint8_t* serial, size_t serialLen,
                                  const uint8_t* sigBuf, size_t sigBufLen,

                                  SonosSignature* sig,
                                  SonosHashCallback h,
                                  SonosRawVerifyCallback v,
                                  SonosKeyLookupCallback l,
                                  const void* lArg,
                                  SonosKeyReleaseCallback r);

int sonosUnlockIsDeviceUnlocked(const struct manufacturing_data_page* mdp,
                                const struct manufacturing_data_page3* mdp3,

                                SonosSignature* sig,
                                SonosHashCallback h,
                                SonosRawVerifyCallback v,
                                SonosKeyLookupCallback lookup,
                                const void* lookupArgCpuid,
                                const void* lookupArgUnlock,
                                SonosKeyReleaseCallback r);

int sonosUnlockIsAuthFeatureEnabled(uint32_t flag,
                                    const struct manufacturing_data_page* mdp,
                                    const struct manufacturing_data_page3* mdp3,

                                    SonosSignature* sig,
                                    SonosHashCallback h,
                                    SonosRawVerifyCallback v,
                                    SonosKeyLookupCallback lookup,
                                    const void* lookupArgCpuid,
                                    const void* lookupArgUnlock,
                                    SonosKeyReleaseCallback r);

uint32_t sonosUnlockGetNextFuseVal(uint32_t fuseval);

#ifdef __cplusplus
}
#endif

#endif
