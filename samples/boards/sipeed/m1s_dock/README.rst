.. zephyr:code-sample:: m1s_dock_triplecore
   :name: M1S Dock Triple-Core Demo
   :relevant-api: kernel

   Run Zephyr on all three BL808 cores (M0, D0, LP) simultaneously.

Overview
********

This sample demonstrates running Zephyr on all three cores of the BL808 SoC
on the Sipeed M1S Dock:

- **M0** (T-Head E907, 320 MHz) — bootloader that releases D0 and LP, then
  forwards their console output to UART0 via XRAM ring buffers.
- **D0** (T-Head C906, 480 MHz) — prints a 1 Hz tick via XRAM console.
- **LP** (T-Head E902, 80 MHz) — prints a 1 Hz tick via XRAM console.

Building and Running
********************

Three separate images must be built and flashed. A helper script is provided::

   # Build and flash all three cores (requires bflb-mcu-tool-uart)
   samples/boards/sipeed/m1s_dock/flash.sh /dev/ttyACM2

Or manually::

   # 1. Build LP app
   west build -p -b m1s_dock/bl808c09q2i/lp samples/boards/sipeed/m1s_dock/lp_app

   # 2. Build D0 app
   west build -p -b m1s_dock/bl808c09q2i/d0 samples/boards/sipeed/m1s_dock/d0_app

   # 3. Build M0 bootloader
   west build -p -b m1s_dock/bl808c09q2i/m0 samples/boards/sipeed/m1s_dock/m0_app

   # Flash order: LP at 0x20000, D0 at 0x100000, M0 last (preserves boot header)
   bflb-mcu-tool-uart --port /dev/ttyACM2 --baudrate 2000000 --chipname bl808 \
       --firmware build_lp/zephyr/zephyr.bin --addr 0x20000
   bflb-mcu-tool-uart --port /dev/ttyACM2 --baudrate 2000000 --chipname bl808 \
       --firmware build_d0/zephyr/zephyr.bin --addr 0x100000
   bflb-mcu-tool-uart --port /dev/ttyACM2 --baudrate 2000000 --chipname bl808 \
       --firmware build_m0/zephyr/zephyr.bin

Sample Output
*************

.. code-block:: console

   [M0] tick 0
   [D0] tick 0
   [LP] tick 0
   [M0] tick 1
   [D0] tick 1
   [LP] tick 1
   [M0] tick 2
   [D0] tick 2
   [LP] tick 2
