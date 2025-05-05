
.. _peripheral_power_benchmark:

Bluetooth: Peripheral Power Benchmark
#####################################

.. contents::
   :local:
   :depth: 2

The Peripheral Power Benchmark is a simple application created to make easier measurements of power usage.

nRF54h20 Radio core executing from local RAM (TCM)
##################################################

There is a possibility to run the radio core firmware from local RAM (TCM).
That needs following manual steps, meaning ordinary `west build` and `west flash` will not work.

.. note::
   This step assumes work on dedicated branches in sdk-nrf, sdk-zephyr, dragoon, and MCUBoot repositories.

1. Build MCUBoot for nRF54h20 radio core in `NCS/bootloader/mcuboot/boot/zephyr/`:

.. code-block:: console

    west build -b nrf54h20dk/nrf54h20/cpurad -d build_54h -p --no-sysbuild

.. note::
   The memory map for radio core must match to memory map used in the sample application.
   The mcuboot may not setup any IPC communication to secdom or sysctrl.
   It will break the application when it is booted from local RAM.
   Current implementation of IPC doesn't allow for re-binding.

2. Build the sample application

.. code-block:: console

   west build -b nrf54h20dk/nrf54h20/cpuapp -d build_54h -p

3. Run `build_54h_ram_hex.sh`.

