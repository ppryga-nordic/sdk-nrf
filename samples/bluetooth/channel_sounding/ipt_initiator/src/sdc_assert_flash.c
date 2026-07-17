/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "sdc_assert_flash.h"

#include <string.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <hal/nrf_rramc.h>
#include <nrfx_rramc.h>

LOG_MODULE_REGISTER(sdc_assert_flash, LOG_LEVEL_INF);

#define ASSERT_MAGIC_WORD      0xA55AB16BUL
#define FLASH_PAGE_SIZE        0x1000U

#if defined(CONFIG_NRF_RRAM_WRITE_BUFFER_SIZE) && CONFIG_NRF_RRAM_WRITE_BUFFER_SIZE > 0
#define WRITE_BUFFER_SIZE CONFIG_NRF_RRAM_WRITE_BUFFER_SIZE
#else
#define WRITE_BUFFER_SIZE 0
#endif

static __attribute__((aligned(FLASH_PAGE_SIZE))) const uint8_t m_flash_assert_page[FLASH_PAGE_SIZE];

static const uint32_t m_page_start = (uint32_t)&m_flash_assert_page;
static const uint32_t m_page_end = m_page_start + FLASH_PAGE_SIZE - 1U;

static void m_mpu_disable(void)
{
#ifdef MPU
	MPU->CTRL &= ~MPU_CTRL_ENABLE_Msk;
#endif
}

static void m_mpu_restore(bool was_enabled)
{
#ifdef MPU
	if (was_enabled) {
		MPU->CTRL |= MPU_CTRL_ENABLE_Msk;
	}
#else
	ARG_UNUSED(was_enabled);
#endif
}

static bool m_mpu_disable_get_state(void)
{
#ifdef MPU
	return (MPU->CTRL & MPU_CTRL_ENABLE_Msk) != 0U;
#else
	return false;
#endif
}

static void m_rram_write(uint32_t addr, const void *src, uint32_t len)
{
#if !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	nrfx_rramc_write_enable_set(true, WRITE_BUFFER_SIZE);
#endif

	nrf_rramc_buffer_write(addr, (void *)src, len);
	barrier_dmem_fence_full();

#if WRITE_BUFFER_SIZE > 0 && !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	nrf_rramc_task_trigger(NRF_RRAMC, NRF_RRAMC_TASK_COMMIT_WRITEBUF);
#endif

#if !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	nrfx_rramc_write_enable_set(false, WRITE_BUFFER_SIZE);
#endif
}

static void m_page_erase(void)
{
#if !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	nrfx_rramc_write_enable_set(true, WRITE_BUFFER_SIZE);
#endif

	memset((void *)m_page_start, 0xFF, FLASH_PAGE_SIZE);
	barrier_dmem_fence_full();

#if !defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
	nrfx_rramc_write_enable_set(false, WRITE_BUFFER_SIZE);
#endif
}

void sdc_assert_flash_report_stored(void)
{
	const uint32_t magic = *(volatile uint32_t *)m_page_start;

	if (magic != ASSERT_MAGIC_WORD) {
		return;
	}

	const uint32_t line = *(volatile uint32_t *)(m_page_start + 4U);
	const uint32_t str_len = *(volatile uint32_t *)(m_page_start + 8U);
	const char *file = (const char *)(m_page_start + 12U);

	if (str_len == 0U || str_len > 32U || (m_page_start + 12U + str_len) > m_page_end) {
		LOG_WRN("Stored SDC assert header is corrupt");
		return;
	}

	LOG_WRN("Stored SDC assert from flash: file %.*s, line 0x%04x", (int)str_len, file, line);
}

void sdc_assert_flash_store(const char *file, uint32_t line)
{
	const uint32_t str_len = strlen(file);
	const uint32_t header[3] = { ASSERT_MAGIC_WORD, line, str_len };
	uint32_t addr = m_page_start;
	unsigned int key = irq_lock();
	bool mpu_was_enabled = m_mpu_disable_get_state();

	if (str_len == 0U || (addr + sizeof(header) + str_len) > m_page_end) {
		m_mpu_restore(mpu_was_enabled);
		irq_unlock(key);
		return;
	}

	m_mpu_disable();
	m_page_erase();
	m_rram_write(addr, header, sizeof(header));
	addr += sizeof(header);
	m_rram_write(addr, file, str_len);
	m_mpu_restore(mpu_was_enabled);
	irq_unlock(key);
}
