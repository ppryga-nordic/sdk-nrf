#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

zephyr_library()

zephyr_library_sources(mpsl_init.c)

zephyr_code_relocate(FILES mpsl_init.c LOCATION RAM)
