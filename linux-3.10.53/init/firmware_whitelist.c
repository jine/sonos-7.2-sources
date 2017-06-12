#include <asm/byteorder.h>
#include <sonos/firmware_whitelist.h>

/*
 * Three sizes need to match up:
 * -the numEntries field here (after the whitelist type)
 * -the number of whitelist entries here
 * -the size of the entries array in firmware_whitelist.h
 *
 * Actually that third one doesn't need to be an exact match to the other
 * two. Its only restriction is that it must be at least as big as the
 * other two and that it probably shouldn't be zero (to avoid declaring a
 * zero element array in C).
 */
const SonosFirmwareWhitelist SONOS_FIRMWARE_WHITELIST =
{
	{
		{ SONOS_FWW_MAGIC_INIT },
		SONOS_FWW_TYPE_CPUID,
		/* numEntries field */
		__cpu_to_be32(0)
	},
#if 0
	/* the cpuid whitelist entries */
	.x.cpuidEntries =
	{
		{ { 0xde, 0x73, 0x0b, 0xdf, 0x12, 0x0c, 0xaa, 0xaa } },
		{ { 0xb8, 0xe9, 0x37, 0x2f, 0xff, 0xf6, 0xaa, 0xaa } },
	}
#endif
};

