/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <errno.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/policy.h>

#include <hal/nrf_cache.h>
#include <hal/nrf_lrcconf.h>
#include <hal/nrf_memconf.h>

#include "ipc_bt.h"

LOG_MODULE_REGISTER(ipc_radio, CONFIG_IPC_RADIO_LOG_LEVEL);

#if !(CONFIG_IPC_RADIO_802154 || CONFIG_IPC_RADIO_BT)
#error "No radio serialization selected."
#endif

#define USE_CACHE_STORE_RESTORE

int main(void)
{
	int err;

	//pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_RAM, PM_ALL_SUBSTATES);
	//pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
#if defined(USE_CACHE_STORE_RESTORE)
	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_0, false);
	nrf_memconf_ramblock_ret_enable_set(NRF_MEMCONF, RAMBLOCK_POWER_ID,
							RAMBLOCK_CONTROL_BIT_DCACHE, true);
	nrf_memconf_ramblock_ret_enable_set(NRF_MEMCONF, RAMBLOCK_POWER_ID,
							RAMBLOCK_CONTROL_BIT_ICACHE, true);
	nrf_memconf_ramblock_ret2_enable_set(NRF_MEMCONF, RAMBLOCK_POWER_ID,
							RAMBLOCK_CONTROL_BIT_DCACHE, true);
	nrf_memconf_ramblock_ret2_enable_set(NRF_MEMCONF, RAMBLOCK_POWER_ID,
							RAMBLOCK_CONTROL_BIT_ICACHE, true);
#endif /* USE_CACHE_STORE_RESTORE */

	err = ipc_bt_init();
	if ((err) && (err != -ENOSYS)) {
		LOG_ERR("Error initializing ipc radio %d", err);
		return err;
	}

	for (;;) {
		err = ipc_bt_process();

		if (err == -ENOSYS) {
			/* Particular implementation does not need the process function */
			return 0;
		} else if (err) {
			LOG_ERR("Error processing ipc radio %d", err);
			return err;
		}
	}
}
