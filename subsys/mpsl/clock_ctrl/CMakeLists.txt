#
# Copyright (c) 2025 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(mpsl_clock_ctrl.c)

zephyr_code_relocate(FILES mpsl_clock_ctrl.c LOCATION RAM)
