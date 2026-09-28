// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2026 Intel Corporation.
//
// These contents may have been developed with support from one or more
// Intel-operated generative artificial intelligence solutions.
//
// Module-resource allocator for standalone ztest applications that do not
// link the real module_adapter/module/generic.c allocator. The calls are
// forwarded to the SOF heap API (rtos/alloc.h) without per-module resource
// tracking, so the suite exercises whichever SOF allocator it links, e.g.
// the real zephyr/lib/alloc.c in its Zephyr sys_heap or native_sim host
// heap (CONFIG_SOF_NATIVE_SIM_HOST_HEAP) flavour.

#include <rtos/alloc.h>
#include <sof/audio/module_adapter/module/generic.h>

#include <stddef.h>
#include <stdint.h>

/**
 * @brief Allocate an aligned buffer for a module from the SOF buffer heap.
 * @param mod Module the memory is allocated for (unused, no tracking).
 * @param size Size in bytes.
 * @param alignment Alignment in bytes, 0 for the default.
 * @return Pointer to the allocated memory or NULL on failure.
 */
void *mod_balloc_align(struct processing_module *mod, size_t size, size_t alignment)
{
	(void)mod;

	return rballoc_align(SOF_MEM_FLAG_USER, size, alignment);
}

/**
 * @brief Allocate memory for a module from the default SOF heap.
 * @param mod Module the memory is allocated for (unused, no tracking).
 * @param flags SOF_MEM_FLAG_* allocation flags.
 * @param size Size in bytes.
 * @param alignment Alignment in bytes, 0 for the default.
 * @return Pointer to the allocated memory or NULL on failure.
 */
void *mod_alloc_ext(struct processing_module *mod, uint32_t flags, size_t size, size_t alignment)
{
	(void)mod;

	return sof_heap_alloc(NULL, flags, size, alignment);
}

/**
 * @brief Free memory allocated with mod_alloc_ext() or mod_balloc_align().
 * @param mod Module the memory was allocated for (unused, no tracking).
 * @param ptr Pointer to free, NULL is ignored.
 * @return Always 0.
 */
int mod_free(struct processing_module *mod, const void *ptr)
{
	(void)mod;

	sof_heap_free(NULL, (void *)ptr);

	return 0;
}
