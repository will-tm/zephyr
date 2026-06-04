#!/bin/bash
# Build and flash all three BL808 cores for the M1S Dock.
# Usage: ./flash.sh <tty>
set -e

TTY="${1:?Usage: $0 <tty>}"
SAMPLE="$(cd "$(dirname "$0")" && pwd)"

echo "=== Building LP ==="
west build -p -b m1s_dock/bl808c09q2i/lp "$SAMPLE/lp_app" -d /tmp/bl808_lp_build

echo "=== Building D0 ==="
west build -p -b m1s_dock/bl808c09q2i/d0 "$SAMPLE/d0_app" -d /tmp/bl808_d0_build

echo "=== Building M0 ==="
west build -p -b m1s_dock/bl808c09q2i/m0 "$SAMPLE/m0_app" -d /tmp/bl808_m0_build

echo "=== Flashing LP at 0x20000 ==="
bflb-mcu-tool-uart --port "$TTY" --baudrate 2000000 --chipname bl808 \
    --firmware /tmp/bl808_lp_build/zephyr/zephyr.bin --addr 0x20000

echo "=== Flashing D0 at 0x100000 ==="
bflb-mcu-tool-uart --port "$TTY" --baudrate 2000000 --chipname bl808 \
    --firmware /tmp/bl808_d0_build/zephyr/zephyr.bin --addr 0x100000

echo "=== Flashing M0 ==="
bflb-mcu-tool-uart --port "$TTY" --baudrate 2000000 --chipname bl808 \
    --firmware /tmp/bl808_m0_build/zephyr/zephyr.bin

echo "=== Done. Monitor: python3 -m serial.tools.miniterm $TTY 115200 --raw ==="
