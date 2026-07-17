/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef SDC_ASSERT_FLASH_H__
#define SDC_ASSERT_FLASH_H__

#include <stdint.h>

/** @brief Log a previously stored SDC assert, if present. */
void sdc_assert_flash_report_stored(void);

/** @brief Store an SDC assert to flash. */
void sdc_assert_flash_store(const char *file, uint32_t line);

#endif /* SDC_ASSERT_FLASH_H__ */
