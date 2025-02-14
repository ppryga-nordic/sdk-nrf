#
# Copyright (c) 2024 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(mpsl_pm_utils.c)

zephyr_code_relocate(FILES mpsl_pm_utils.c LOCATION RAM)