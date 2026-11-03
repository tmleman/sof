/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2026 Intel Corporation.
 */

/**
 * \file platform/lib/dai_stub.h
 * \brief Stub Zephyr DAI devices for IPC fuzzing on native_sim.
 *
 * The native_sim devicetree provides no DAI nodes, so the device list in
 * src/lib/dai.c is empty and every IPC-created DAI endpoint would fail in
 * dai_get() with -ENODEV. The stub devices defined in
 * src/platform/posix/dai_stub.c fill that gap for fuzz builds; this header
 * exposes the lookup used by dai_get_device() as a fallback.
 */

#ifndef __PLATFORM_LIB_DAI_STUB_H__
#define __PLATFORM_LIB_DAI_STUB_H__

#include <zephyr/device.h>
#include <stdint.h>

/**
 * \brief Look up a stub DAI device by Zephyr DAI type and index.
 *
 * \param[in] z_type Zephyr DAI type (enum dai_type) to match.
 * \param[in] index DAI index to match.
 * \return Matching stub device, or NULL when none matches.
 */
const struct device *sof_fuzz_dai_stub_get_device(int z_type, uint32_t index);

#endif /* __PLATFORM_LIB_DAI_STUB_H__ */
