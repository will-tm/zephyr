#!/bin/bash
# Build and flash CoreMark on all three BL808 cores for the M1S Dock.
# Usage: ./flash.sh <tty>
set -e

TTY="${1:?Usage: $0 <tty>}"
SAMPLE="$(cd "$(dirname "$0")" && pwd)"

echo "=== Building LP ==="
west build -p -b m1s_dock/bl808c09q2i/lp "$SAMPLE/lp_app" -d /tmp/cm_lp

echo "=== Building D0 ==="
west build -p -b m1s_dock/bl808c09q2i/d0 "$SAMPLE/d0_app" -d /tmp/cm_d0

echo "=== Building M0 ==="
west build -p -b m1s_dock/bl808c09q2i/m0 "$SAMPLE/m0_app" -d /tmp/cm_m0

echo "=== Flashing LP at 0x20000 ==="
bflb-mcu-tool-uart --port "$TTY" --baudrate 2000000 --chipname bl808 \
    --firmware /tmp/cm_lp/zephyr/zephyr.bin --addr 0x20000

echo "=== Flashing D0 at 0x100000 ==="
bflb-mcu-tool-uart --port "$TTY" --baudrate 2000000 --chipname bl808 \
    --firmware /tmp/cm_d0/zephyr/zephyr.bin --addr 0x100000

echo "=== Flashing M0 ==="
bflb-mcu-tool-uart --port "$TTY" --baudrate 2000000 --chipname bl808 \
    --firmware /tmp/cm_m0/zephyr/zephyr.bin

echo "=== Done. Monitor: python3 -m serial.tools.miniterm $TTY 115200 --raw ==="
