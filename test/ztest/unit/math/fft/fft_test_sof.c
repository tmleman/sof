// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2026 Intel Corporation.
//
// These contents may have been developed with support from one or more
// Intel-operated generative artificial intelligence solutions.
//
// Minimal SOF context for the standalone FFT ztest application, which
// does not link src/init/init.c. Memory allocation is provided by the
// real zephyr/lib/alloc.c and test/ztest/unit/common/mod_alloc.c.

#include <rtos/sof.h>
#include <sof/audio/component_ext.h>

#include <stdbool.h>

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
