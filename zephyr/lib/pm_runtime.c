// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright(c) 2022 Intel Corporation. All rights reserved.

#include <sof/ipc/common.h>
#include <sof/lib/pm_runtime.h>
#include <sof/lib/uuid.h>
#include <sof/trace/trace.h>
#include <user/trace.h>
#include <sof_versions.h>
#include <stdint.h>
#include <zephyr/pm/policy.h>
#include <sof/ipc/driver.h>

LOG_MODULE_REGISTER(power, 3);

/* 76cc9773-440c-4df9-95a8-72defe7796fc */
DECLARE_SOF_UUID("power", power_uuid, 0x76cc9773, 0x440c, 0x4df9,
		 0x95, 0xa8, 0x72, 0xde, 0xfe, 0x77, 0x96, 0xfc);

DECLARE_TR_CTX(power_tr, SOF_UUID(power_uuid), 3);

static uint32_t d0i3_count = 1;
static int32_t d0i3_low_ticks = INT32_MAX;
static int32_t d0i3_high_ticks = 1;

static uint32_t idle_count = 1;
static int32_t idle_low_ticks = INT32_MAX;
static int32_t idle_high_ticks = 1;

#if defined(CONFIG_PM_POLICY_CUSTOM)
const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
	unsigned int num_cpu_states;
	const struct pm_state_info *cpu_states;

	num_cpu_states = pm_state_cpu_get_all(cpu, &cpu_states);

	for (int i = num_cpu_states - 1; i >= 0; i--) {
		const struct pm_state_info *state = &cpu_states[i];
		uint32_t min_residency, exit_latency;

		/* policy cannot lead to D3 */
		if (state->state == PM_STATE_SOFT_OFF)
			continue;

		/* check if there is a lock on state + substate */
		if (pm_policy_state_lock_is_active(state->state, state->substate_id))
			continue;

		/* checking conditions for D0i3 */
		if (state->state == PM_STATE_RUNTIME_IDLE) {
			/* skipping when secondary cores are active */
			if (cpu_enabled_cores() & ~BIT(PLATFORM_PRIMARY_CORE_ID))
				continue;

			/* skipping when some ipc task is not finished */
			if (ipc_get()->task_mask || !ipc_platform_poll_is_host_ready())
				continue;
		}

		min_residency = k_us_to_ticks_ceil32(state->min_residency_us);
		exit_latency = k_us_to_ticks_ceil32(state->exit_latency_us);

		if (ticks == K_TICKS_FOREVER ||
		    (ticks >= (min_residency + exit_latency))) {
			/* TODO: PM_STATE_RUNTIME_IDLE requires substates to be defined
			 * to handle case with enabled PG andf disabled CG.
			 */
			tr_dbg(&power_tr, "cpu %x, ticks = %d, state counts = %d",
			       cpu, ticks, num_cpu_states);
			tr_dbg(&power_tr, "transition to state %x (min_residency = %u, exit_latency = %u)",
			       state->state, min_residency, exit_latency);
			d0i3_count += 1;
			if (ticks < d0i3_low_ticks)
				d0i3_low_ticks = ticks;

			if (ticks > d0i3_high_ticks)
				d0i3_high_ticks = ticks;

			return state;
		}
	}

	if (!pm_policy_state_lock_is_active(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES)) {
		idle_count += 1;
		if (ticks < idle_low_ticks)
			idle_low_ticks = ticks;

		if (ticks > idle_high_ticks)
			idle_high_ticks = ticks;
	}

	return NULL;
}
#endif /* CONFIG_PM_POLICY_CUSTOM */

void platform_pm_runtime_enable(uint32_t context, uint32_t index)
{
	switch (context) {
	case PM_RUNTIME_DSP:
		pm_policy_state_lock_put(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
		tr_info(&power_tr, "removing prevent on d0i3 (lock is active=%d)",
			pm_policy_state_lock_is_active(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES));
		d0i3_count = 0;
		d0i3_low_ticks = INT32_MAX;
		d0i3_high_ticks = 0;
		idle_count = 0;
		idle_low_ticks = INT32_MAX;
		idle_high_ticks = 0;
		break;
	default:
		break;
	}
}

void platform_pm_runtime_disable(uint32_t context, uint32_t index)
{
	switch (context) {
	case PM_RUNTIME_DSP:
		tr_info(&power_tr, "putting prevent on d0i3");
			pm_policy_state_lock_get(PM_STATE_RUNTIME_IDLE, PM_ALL_SUBSTATES);
		tr_info(&power_tr, "PG stats[D0i3]: count %u, ticks: low = %d, high = %d",
			d0i3_count, d0i3_low_ticks, d0i3_high_ticks);
		tr_info(&power_tr, "PG stats[IDLE]: count %u, ticks: low = %d, high = %d",
			idle_count, idle_low_ticks, idle_high_ticks);
		break;
	default:
		break;
	}
}

void platform_pm_runtime_init(struct pm_runtime_data *prd)
{ }

void platform_pm_runtime_get(enum pm_runtime_context context, uint32_t index,
			     uint32_t flags)
{ }

void platform_pm_runtime_put(enum pm_runtime_context context, uint32_t index,
			     uint32_t flags)
{ }

void platform_pm_runtime_prepare_d0ix_en(uint32_t index)
{ }

void platform_pm_runtime_power_off(void)
{ }
