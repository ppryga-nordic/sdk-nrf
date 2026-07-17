#
# Copyright (c) 2026 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(channel_sound_ipt_initiator)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_IPT_INITIATOR_SDC_ASSERT_OUTPUT_FLASH app PRIVATE src/sdc_assert_flash.c)
# NORDIC SDK APP END
