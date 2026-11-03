// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2026 Intel Corporation.

/**
 * \file
 * \brief Stub Zephyr DAI devices for IPC fuzzing on native_sim.
 *
 * SOF builds with CONFIG_ZEPHYR_NATIVE_DRIVERS resolve DAIs through the
 * Zephyr device model: dai_get() matches a requested (type, index) pair
 * against devices enumerated from the devicetree. The native_sim board has
 * no DAI nodes, so the lookup always fails and the whole DAI endpoint
 * surface (DAI gateway copiers, dai-zephyr.c, pipeline streaming) is
 * unreachable by the fuzzer.
 *
 * This file registers a set of stub DAI devices (SSP, DMIC, HDA, ALH and
 * UAOL at indices 0 and 1) implementing the minimum Zephyr DAI driver API
 * needed by src/audio/dai-zephyr.c. The stubs accept any configuration and
 * report zeroed properties; config_get returns zero rate/channels, which
 * dai_verify_params() interprets as "hardware accepts any value".
 *
 * config_set deliberately reads every byte of the host-supplied bespoke
 * configuration blob so that an over-declared blob length is caught by the
 * sanitizers at the driver boundary, like a real driver parsing it would.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/dai.h>
#include <platform/lib/dai_stub.h>
#include <string.h>

/** \brief Runtime state of one stub DAI device. */
struct fuzz_dai_data {
	/** Immutable identity (type, dai_index) matched by dai_get_device() */
	struct dai_config cfg;
};

/** Checksum sink forcing the bespoke config blob reads to happen. */
static volatile uint8_t fuzz_dai_blob_sum;

/** \brief Power up the stub DAI (no-op). */
static int fuzz_dai_probe(const struct device *dev)
{
	return 0;
}

/** \brief Power down the stub DAI (no-op). */
static int fuzz_dai_remove(const struct device *dev)
{
	return 0;
}

/**
 * \brief Accept any DAI configuration.
 *
 * Reads the whole bespoke blob so sanitizers validate the host-controlled
 * (pointer, size) pair, mirroring a real driver parsing the blob.
 */
static int fuzz_dai_config_set(const struct device *dev, const struct dai_config *cfg,
			       const void *bespoke_cfg, size_t size)
{
	const uint8_t *bytes = bespoke_cfg;
	size_t i;

	for (i = 0; bytes && i < size; i++)
		fuzz_dai_blob_sum += bytes[i];

	return 0;
}

/**
 * \brief Report the stub identity with unconstrained stream parameters.
 *
 * Zero rate/channels mean "any value is acceptable" to
 * dai_verify_params() in dai-zephyr.c.
 */
static int fuzz_dai_config_get(const struct device *dev, struct dai_config *cfg,
			       enum dai_dir dir)
{
	struct fuzz_dai_data *data = dev->data;

	*cfg = data->cfg;

	return 0;
}

/** \brief Fill in zeroed DAI properties, echoing the requested stream ID. */
static int fuzz_dai_get_properties_copy(const struct device *dev, enum dai_dir dir,
					int stream_id, struct dai_properties *dst)
{
	memset(dst, 0, sizeof(*dst));
	dst->stream_id = stream_id;

	return 0;
}

/** \brief Accept any trigger command (no-op). */
static int fuzz_dai_trigger(const struct device *dev, enum dai_dir dir,
			    enum dai_trigger_cmd cmd)
{
	return 0;
}

/** \brief Accept timestamp configuration (no-op). */
static int fuzz_dai_ts_config(const struct device *dev, struct dai_ts_cfg *cfg)
{
	return 0;
}

/** \brief Start timestamping (no-op). */
static int fuzz_dai_ts_start(const struct device *dev, struct dai_ts_cfg *cfg)
{
	return 0;
}

/** \brief Stop timestamping (no-op). */
static int fuzz_dai_ts_stop(const struct device *dev, struct dai_ts_cfg *cfg)
{
	return 0;
}

/** \brief Return a zeroed timestamp sample. */
static int fuzz_dai_ts_get(const struct device *dev, struct dai_ts_cfg *cfg,
			   struct dai_ts_data *tsd)
{
	memset(tsd, 0, sizeof(*tsd));

	return 0;
}

static DEVICE_API(dai, fuzz_dai_api) = {
	.probe = fuzz_dai_probe,
	.remove = fuzz_dai_remove,
	.config_set = fuzz_dai_config_set,
	.config_get = fuzz_dai_config_get,
	.get_properties_copy = fuzz_dai_get_properties_copy,
	.trigger = fuzz_dai_trigger,
	.ts_config = fuzz_dai_ts_config,
	.ts_start = fuzz_dai_ts_start,
	.ts_stop = fuzz_dai_ts_stop,
	.ts_get = fuzz_dai_ts_get,
};

/** \brief Define one stub DAI device with a fixed (type, index) identity. */
#define FUZZ_DAI_DEVICE(dev_id, ztype, idx)					\
	static struct fuzz_dai_data fuzz_dai_data_##dev_id = {			\
		.cfg = {							\
			.type = (ztype),					\
			.dai_index = (idx),					\
		},								\
	};									\
	DEVICE_DEFINE(dev_id, #dev_id, NULL, NULL,				\
		      &fuzz_dai_data_##dev_id, NULL, POST_KERNEL,		\
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &fuzz_dai_api)

FUZZ_DAI_DEVICE(fuzz_dai_ssp0, DAI_INTEL_SSP, 0);
FUZZ_DAI_DEVICE(fuzz_dai_ssp1, DAI_INTEL_SSP, 1);
FUZZ_DAI_DEVICE(fuzz_dai_dmic0, DAI_INTEL_DMIC, 0);
FUZZ_DAI_DEVICE(fuzz_dai_dmic1, DAI_INTEL_DMIC, 1);
FUZZ_DAI_DEVICE(fuzz_dai_hda0, DAI_INTEL_HDA, 0);
FUZZ_DAI_DEVICE(fuzz_dai_hda1, DAI_INTEL_HDA, 1);
FUZZ_DAI_DEVICE(fuzz_dai_alh0, DAI_INTEL_ALH, 0);
FUZZ_DAI_DEVICE(fuzz_dai_alh1, DAI_INTEL_ALH, 1);
FUZZ_DAI_DEVICE(fuzz_dai_uaol0, DAI_INTEL_UAOL, 0);
FUZZ_DAI_DEVICE(fuzz_dai_uaol1, DAI_INTEL_UAOL, 1);

/* DEVICE_DEFINE() gives non-devicetree devices internal linkage, so the
 * stubs cannot be spliced into the zephyr_dev[] initializer in
 * src/lib/dai.c; expose them through a lookup instead.
 */
static const struct device *const fuzz_dai_devs[] = {
	DEVICE_GET(fuzz_dai_ssp0),
	DEVICE_GET(fuzz_dai_ssp1),
	DEVICE_GET(fuzz_dai_dmic0),
	DEVICE_GET(fuzz_dai_dmic1),
	DEVICE_GET(fuzz_dai_hda0),
	DEVICE_GET(fuzz_dai_hda1),
	DEVICE_GET(fuzz_dai_alh0),
	DEVICE_GET(fuzz_dai_alh1),
	DEVICE_GET(fuzz_dai_uaol0),
	DEVICE_GET(fuzz_dai_uaol1),
};

const struct device *sof_fuzz_dai_stub_get_device(int z_type, uint32_t index)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(fuzz_dai_devs); i++) {
		const struct fuzz_dai_data *data = fuzz_dai_devs[i]->data;

		if (data->cfg.type == z_type && data->cfg.dai_index == index)
			return fuzz_dai_devs[i];
	}

	return NULL;
}
