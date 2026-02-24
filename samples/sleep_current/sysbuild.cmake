# This is a workaround for a workaround:
# - Power management configurations are not enabled by default in NCS (SRPMT-1274)
# - Dragoon verification build system default overlays are not applied for
#   the non-default core. Therefore we apply them here.

# if (${SB_CONFIG_SOC_NRF54H20_CPURAD})
#   set(empty_app_core_OVERLAY_CONFIG  ${default_overlay_dir}/default_overlay_nrf54h20dk_nrf54h20_cpuapp.conf CACHE INTERNAL "Remote image config")
# endif()
