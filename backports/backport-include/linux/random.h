#ifndef __BACKPORT_RANDOM_H
#define __BACKPORT_RANDOM_H
#include_next <linux/random.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0) && LINUX_VERSION_CODE < KERNEL_VERSION(3,4,10)) || \
    (LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0) && LINUX_VERSION_CODE < KERNEL_VERSION(3,2,27)) || \
    LINUX_VERSION_CODE < KERNEL_VERSION(3,0,41)
#define add_device_randomness LINUX_BACKPORT(add_device_randomness)
static inline void add_device_randomness(const void *buf, unsigned int size)
{
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0)
/**
 * prandom_u32_max - returns a pseudo-random number in interval [0, ep_ro)
 * @ep_ro: right open interval endpoint
 *
 * Returns a pseudo-random number that is in interval [0, ep_ro). Note
 * that the result depends on PRNG being well distributed in [0, ~0U]
 * u32 space. Here we use maximally equidistributed combined Tausworthe
 * generator, that is, prandom_u32(). This is useful when requesting a
 * random index of an array containing ep_ro elements, for example.
 *
 * Returns: pseudo-random number in interval [0, ep_ro)
 */
#define prandom_u32_max LINUX_BACKPORT(prandom_u32_max)
static inline u32 prandom_u32_max(u32 ep_ro)
{
	return (u32)(((u64) prandom_u32() * ep_ro) >> 32);
}
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0) */

#endif /* __BACKPORT_RANDOM_H */
