#ifndef _SONOS_BYTEORDER_H
#define _SONOS_BYTEORDER_H

/*
 * sonos/byteorder.h
 * CPU byte order support
 *
 * Sonos has hardware platforms that are either little-endian or big-endian. 
 * On any platform big or little endian data may need to be processed.
 *
 * This manages the CPU side of the problem creating CPU_TO_XX32(x) and
 * XX32_TO_CPU(x) familes of defines.
 * An XX may be either BE - big-endian or LE - little-endian referring
 * data that is either big or little endian.
 * The basic set of defines is:
 *      CPU_TO_BE32(x), CPU_TO_BE16(x) and CPU_TO_BE64(x)
 *      BE32_TO_CPU(x), BE16_TO_CPU(x) and BE64_TO_CPU(x)
 * to translate between 32, 16 and 64 bit big-endian data and cpu binary.  
 * It also creates
 *      CPU_TO_LE32(x), CPU_TO_LE16(x) and CPU_TO_LE64(x)
 *      LE32_TO_CPU(x), LE16_TO_CPU(x) and LE64_TO_CPU(x)
 * to translate between 32, 16 and 64 bit little-endian data and cpu binary.
 *
 * For convience there are also CPU_TO_XX32_A(ax) and XX32_TO_CPU_A(x)
 * varients of each of the above defines to be used when the parameter
 * is the address of a 32, 16 or 64 bit variable.
 *
 * and finally a CPU_TO_XX32_P(x) and XX32_TO_CPU_P(x) set of varients to
 * take an address of a 32, 16 or 64 bit variable and translate it in
 * place.
 *
 */

#include <netinet/in.h>
#ifdef __SONOS_SH4__
#include <sonos/asm-sh-byteorder.h>
#endif
#ifdef __SONOS_PPC__
#if defined(SONOS_ARCH_LIMELIGHT) || defined(SONOS_ARCH_FENWAY)
#include <asm/byteorder.h>
#else
#include <sonos/asm-ppc-byteorder.h>
#endif
#endif
#ifdef __SONOS_I386__
#include <sonos/asm-i386-byteorder.h>
#endif
#if defined(__SONOS_MIPS__) || defined(__SONOS_FILLMORE__)
#include <sonos/asm-mips-byteorder.h>
#endif
#ifdef __SONOS_ARM__
#include <sonos/asm-arm-byteorder.h>
#endif

/*
 * Do the prototypes. Somebody might want to take the
 * address or some such sick thing..
 */
extern __u32			CPU_TO_BE32(__u32);
extern __u16	                CPU_TO_BE16(__u16);
extern __u64			CPU_TO_BE64(__u64);

extern __u32			BE32_TO_CPU(__u32);
extern __u16            	BE16_TO_CPU(__u16);
extern __u64			BE64_TO_CPU(__u64);

extern __u32			CPU_TO_LE32(__u32);
extern __u16	                CPU_TO_LE16(__u16);
extern __u64			CPU_TO_LE64(__u64);

extern __u32			LE32_TO_CPU(__u32);
extern __u16            	LE16_TO_CPU(__u16);
extern __u64			LE64_TO_CPU(__u64);

/*
 * Translate between cpu and big-endian data
 */
#define CPU_TO_BE32(x)          __cpu_to_be32(x)
#define CPU_TO_BE16(x)          __cpu_to_be16(x)
#define CPU_TO_BE64(x)          __cpu_to_be64(x)

#define BE32_TO_CPU(x)          __be32_to_cpu(x)
#define BE16_TO_CPU(x)          __be16_to_cpu(x)
#define BE64_TO_CPU(x)          __be64_to_cpu(x)

/*
 * Translate between cpu and little-endian data
 */
#define CPU_TO_LE32(x)          __cpu_to_le32(x)
#define CPU_TO_LE16(x)          __cpu_to_le16(x)
#define CPU_TO_LE64(x)          __cpu_to_le64(x)

#define LE32_TO_CPU(x)          __le32_to_cpu(x)
#define LE16_TO_CPU(x)          __le16_to_cpu(x)
#define LE64_TO_CPU(x)          __le64_to_cpu(x)

/*
 * Param is an ADDRESS variant
 */

extern __u32			CPU_TO_BE32p(__u32);
extern __u16	                CPU_TO_BE16p(__u16);
extern __u64			CPU_TO_BE64p(__u64);

extern __u32			BE32_TO_CPUp(__u32);
extern __u16            	BE16_TO_CPUp(__u16);
extern __u64			BE64_TO_CPUp(__u64);

extern __u32			CPU_TO_LE32p(__u32);
extern __u16	                CPU_TO_LE16p(__u16);
extern __u64			CPU_TO_LE64p(__u64);

extern __u32			LE32_TO_CPUp(__u32);
extern __u16            	LE16_TO_CPUp(__u16);
extern __u64			LE64_TO_CPUp(__u64);

/*
 * Translate between cpu and big-endian data
 */
#define CPU_TO_BE32_A(x)          __cpu_to_be32p(x)
#define CPU_TO_BE16_A(x)          __cpu_to_be16p(x)
#define CPU_TO_BE64_A(x)          __cpu_to_be64p(x)

#define BE32_TO_CPU_A(x)          __be32_to_cpup(x)
#define BE16_TO_CPU_A(x)          __be16_to_cpup(x)
#define BE64_TO_CPU_A(x)          __be64_to_cpup(x)

/*
 * Translate between cpu and little-endian data
 */
#define CPU_TO_LE32_A(x)          __cpu_to_le32p(x)
#define CPU_TO_LE16_A(x)          __cpu_to_le16p(x)
#define CPU_TO_LE64_A(x)          __cpu_to_le64p(x)

#define LE32_TO_CPU_A(x)          __le32_to_cpup(x)
#define LE16_TO_CPU_A(x)          __le16_to_cpup(x)
#define LE64_TO_CPU_A(x)          __le64_to_cpup(x)

/*
 * Operates on param as an ADDRESS changing the data in place
 */

extern void			CPU_TO_BE32s(__u32);
extern void	                CPU_TO_BE16s(__u16);
extern void			CPU_TO_BE64s(__u64);

extern void			BE32_TO_CPUs(__u32);
extern void             	BE16_TO_CPUs(__u16);
extern void			BE64_TO_CPUs(__u64);

extern void			CPU_TO_LE32s(__u32);
extern void	                CPU_TO_LE16s(__u16);
extern void			CPU_TO_LE64s(__u64);

extern void			LE32_TO_CPUs(__u32);
extern void             	LE16_TO_CPUs(__u16);
extern void			LE64_TO_CPUs(__u64);

/*
 * Translate between cpu and big-endian data
 */
#define CPU_TO_BE32_P(x)          __cpu_to_be32s(x)
#define CPU_TO_BE16_P(x)          __cpu_to_be16s(x)
#define CPU_TO_BE64_P(x)          __cpu_to_be64s(x)

#define BE32_TO_CPU_P(x)          __be32_to_cpus(x)
#define BE16_TO_CPU_P(x)          __be16_to_cpus(x)
#define BE64_TO_CPU_P(x)          __be64_to_cpus(x)

/*
 * Translate between cpu and little-endian data
 */
#define CPU_TO_LE32_P(x)          __cpu_to_le32s(x)
#define CPU_TO_LE16_P(x)          __cpu_to_le16s(x)
#define CPU_TO_LE64_P(x)          __cpu_to_le64s(x)

#define LE32_TO_CPU_P(x)          __le32_to_cpus(x)
#define LE16_TO_CPU_P(x)          __le16_to_cpus(x)
#define LE64_TO_CPU_P(x)          __le64_to_cpus(x)

#endif /* _SONOS_BYTEORDER_H */
