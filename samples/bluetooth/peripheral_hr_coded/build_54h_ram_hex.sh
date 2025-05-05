#!/bin/bash
set -e

#the script expects the MCUBoot build location and the sample build location

WORK_DIR=$(pwd)
BUILD_DIR=build_54h_ram
RADIO_APP_BIN_DIR=$WORK_DIR/$BUILD_DIR/ipc_radio/zephyr
RADIO_APP_BIN_PATH=$RADIO_APP_BIN_DIR/zephyr.signed.bin

echo "Current dir: $WORK_DIR"

if [ ! -d "$RADIO_APP_BIN_DIR" ]; then
  echo "Sample application build directory doesn't exist: $RADIO_APP_BIN_DIR"
  exit 1
fi

MCUBOOT_NCS_DIR=bootloader/mcuboot/boot/zephyr
MCUBOOT_BUILD_DIR=$WORK_DIR/../../../../$MCUBOOT_NCS_DIR/$BUILD_DIR

if [ ! -d "$MCUBOOT_BUILD_DIR" ]; then
  echo "MCUBoot application build directory doesn't exist: $MCUBOOT_BUILD_DIR"
  exit 1
fi

echo "MCUBoot build dir $MCUBOOT_BUILD_DIR"

# temporary out
TEMP_BIN_DIR=$WORK_DIR/temp_bin_54h20_ram

mkdir -p $TEMP_BIN_DIR

RADIO_SIGNED_APP_HEX_PATH=$TEMP_BIN_DIR/nrf54h20dk_nrf54h20_cpurad.signed.hex

# bin2hex is part of IntelHex module
bin2hex.py --offset=0xe064000 $RADIO_APP_BIN_PATH $RADIO_SIGNED_APP_HEX_PATH
if [ $? -ne 0 ]; then
    echo "Failed to create the singed binary application hex file: error $status"
fi

MCUBOOT_HEX_PATH=$MCUBOOT_BUILD_DIR/zephyr/zephyr.hex
RADIO_APP_UICR_HEX_PATH=$RADIO_APP_BIN_DIR/uicr.hex
RADIO_FW_FINAL_HEX_PATH=$TEMP_BIN_DIR/radio_uicr_merged.hex

mergehex -m $MCUBOOT_HEX_PATH $RADIO_APP_UICR_HEX_PATH $RADIO_SIGNED_APP_HEX_PATH -o $RADIO_FW_FINAL_HEX_PATH
if [ $? -ne 0 ]; then
    echo "Failed to create final merged radio FW: error $status"
fi

DK_SEGGER_ID=1051168694

# Cleanup DK
nrfutil device x-boot-mode-set --boot-mode safe --serial-number $DK_SEGGER_ID --log-level debug
if [ $? -ne 0 ]; then
    echo "Failed to enter safe boot mode: error $status"
fi

nrfutil device erase --all --log-level trace --core Application --serial-number $DK_SEGGER_ID
if [ $? -ne 0 ]; then
    echo "Failed to erase Application core: error $status"
fi
nrfutil device erase --all --log-level trace --core Network --serial-number $DK_SEGGER_ID
if [ $? -ne 0 ]; then
    echo "Failed to erase Radio core: error $status"
fi

nrfutil device x-boot-mode-set --boot-mode normal --serial-number $DK_SEGGER_ID
if [ $? -ne 0 ]; then
    echo "Failed to enter normal boot mode: error $status"
fi

# Flash SUIT manifest starter instead of the application suit manifes.
# The started manifest doesn't check the memory map so you don't need to aligne it.
nrfutil device program --firmware $WORK_DIR/../../../../modules/hal/nordic/zephyr/blobs/suit/bin/suit_manifest_starter.hex --serial-number $DK_SEGGER_ID
if [ $? -ne 0 ]; then
    echo "Failed to program starter suit manifest: error $status"
fi

# Flashing firmware
nrfutil device program --options chip_erase_mode=ERASE_NONE --firmware $RADIO_FW_FINAL_HEX_PATH --core Network --serial-number $DK_SEGGER_ID
if [ $? -ne 0 ]; then
    echo "Failed to program radio core firmware: error $?"
fi

APPLICATION_APP_BIN_DIR=$WORK_DIR/$BUILD_DIR/peripheral_hr_coded/zephyr
APPLICATION_APP_HEX_PATH=$APPLICATION_APP_BIN_DIR/uicr_merged.hex

nrfutil device program --options chip_erase_mode=ERASE_NONE --firmware $APPLICATION_APP_HEX_PATH --core Application --serial-number $DK_SEGGER_ID
if [ $? -ne 0 ]; then
    echo "Failed to program application core firmware: error $?"
fi

# Reset DK
nrfutil device reset --reset-kind RESET_PIN --core Secure --serial-number $DK_SEGGER_ID

exit 0