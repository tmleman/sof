// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2026 Intel Corporation.
//
// These contents may have been developed with support from one or more
// Intel-operated generative artificial intelligence solutions.
//
// Minimal module-resource allocator mocks for the standalone FFT-multi
// Ztest application. Mirrors the WEAK fallbacks provided for the legacy
// CMocka build by test/cmocka/src/common_mocks.c, since this suite does
// not link the real module_adapter/module/generic.c allocator.

#include <sof/audio/module_adapter/module/generic.h>

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * @brief Allocate aligned memory for a module, bypassing the resource
 * tracking of the real module_adapter allocator (not linked into this
 * suite). Mirrors the WEAK fallback used by the legacy CMocka test.
 */
void *mod_balloc_align(struct processing_module *mod, size_t size, size_t alignment)
{
	(void)mod;
	(void)alignment;

	return malloc(size);
}

/**
 * @brief Allocate memory for a module, bypassing the resource tracking of
 * the real module_adapter allocator. Mirrors the WEAK fallback used by the
 * legacy CMocka test.
 */
void *mod_alloc_ext(struct processing_module *mod, uint32_t flags, size_t size, size_t alignment)
{
	(void)mod;
	(void)flags;
	(void)alignment;

	return malloc(size);
}

/**
 * @brief Free memory allocated with mod_alloc_ext()/mod_balloc_align().
 */
int mod_free(struct processing_module *mod, const void *ptr)
{
	(void)mod;

	free((void *)ptr);

	return 0;
}
