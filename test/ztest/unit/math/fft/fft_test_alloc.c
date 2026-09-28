// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2026 Intel Corporation.
//
// These contents may have been developed with support from one or more
// Intel-operated generative artificial intelligence solutions.
//
// Minimal allocator/component-resource mocks for the standalone FFT Ztest
// application. Mirrors the WEAK fallbacks provided for the legacy CMocka
// build by test/cmocka/src/common_mocks.c, since this suite does not link
// the real module_adapter/module/generic.c allocator.

#include <rtos/alloc.h>
#include <rtos/sof.h>
#include <sof/audio/component_ext.h>
#include <sof/audio/module_adapter/module/generic.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static struct sof sof_context;
static bool sof_context_initialized;

/**
 * @brief Return the minimal SOF context used by the FFT Ztest application.
 */
struct sof *sof_get(void)
{
	if (!sof_context_initialized) {
		sys_comp_init(&sof_context);
		sof_context_initialized = true;
	}

	return &sof_context;
}

/**
 * @brief Allocate aligned runtime memory for the standalone test application.
 */
void *rmalloc_align(uint32_t flags, size_t bytes, uint32_t alignment)
{
	(void)flags;
	(void)alignment;

	return malloc(bytes);
}

/**
 * @brief Allocate runtime memory for the standalone test application.
 */
void *rmalloc(uint32_t flags, size_t bytes)
{
	(void)flags;

	return malloc(bytes);
}

/**
 * @brief Allocate zero-initialized runtime memory for the test application.
 */
void *rzalloc(uint32_t flags, size_t bytes)
{
	(void)flags;

	return calloc(bytes, 1);
}

/**
 * @brief Allocate aligned buffer memory for the standalone test application.
 */
void *rballoc_align(uint32_t flags, size_t bytes, uint32_t alignment)
{
	(void)flags;
	(void)alignment;

	return malloc(bytes);
}

/**
 * @brief Release memory allocated by the standalone test application.
 */
void rfree(void *ptr)
{
	free(ptr);
}

/**
 * @brief Allocate memory from the test application's heap abstraction.
 */
void *sof_heap_alloc(struct k_heap *heap, uint32_t flags, size_t bytes, size_t alignment)
{
	(void)heap;
	(void)flags;
	(void)alignment;

	return malloc(bytes);
}

/**
 * @brief Release memory allocated through the test heap abstraction.
 */
void sof_heap_free(struct k_heap *heap, void *addr)
{
	(void)heap;

	free(addr);
}

/**
 * @brief Return the absent user heap in the standalone test application.
 */
struct k_heap *sof_sys_user_heap_get(void)
{
	return NULL;
}

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
