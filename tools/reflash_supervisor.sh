#!/bin/bash
# Flash the supervisor MCU over USB, from the machine it's plugged into (the
# desk Pi). Copy it there with the other tools:
#
#   rsync -az bms/tools/reflash_supervisor.sh rpi@10.131.131.50:~/
#   rsync -z supervisor/build/supervisor.uf2 rpi@10.131.131.50:/tmp
#   ssh rpi@10.131.131.50 ./reflash_supervisor.sh [image.uf2] [--backup]
#
# The image defaults to /tmp/supervisor.uf2. --backup first saves the running
# firmware to ~/supervisor_backup_<date>.uf2 (flash it back the same way).
#
# The running firmware is rebooted into BOOTSEL through its USB reset interface
# (picotool -f), so no button is needed. CTR_EN goes low while the supervisor
# reboots, so the contactor drivers are disabled until the new firmware's
# checks pass: don't do this with the contactors closed.
#
# With several supervisors attached, pick one with SUPERVISOR_SERIAL=<serial>
# (from /dev/serial/by-id/usb-S3XY-BMS_Supervisor_<serial>-if00).

set -euo pipefail

UF2=/tmp/supervisor.uf2
BACKUP=0
for arg in "$@"; do
    case "$arg" in
        --backup) BACKUP=1 ;;
        *) UF2=$arg ;;
    esac
done

die() { echo "Error: $*" >&2; exit 1; }

[ -f "$UF2" ] || die "no image at $UF2"

# Refuse anything but a supervisor image: BMS firmware on the supervisor MCU
# halts at boot, and might not come back on USB
info=$(picotool info "$UF2" 2>&1) || die "picotool can't read $UF2: $info"
echo "$info" | grep -q "name: *supervisor$" || die "$UF2 isn't a supervisor image:
$info"
echo "Image: $UF2"
echo "$info" | grep -E "description|binary end" || true

if [ -z "${SUPERVISOR_SERIAL:-}" ]; then
    ports=(/dev/serial/by-id/usb-S3XY-BMS_Supervisor_*-if00)
    [ -e "${ports[0]}" ] || die "no supervisor on USB (/dev/serial/by-id/usb-S3XY-BMS_Supervisor_*). If it is stuck in BOOTSEL, set SUPERVISOR_SERIAL."
    [ ${#ports[@]} -eq 1 ] || die "several supervisors attached, set SUPERVISOR_SERIAL: ${ports[*]}"
    SUPERVISOR_SERIAL=$(basename "${ports[0]}" | sed -E 's/^usb-S3XY-BMS_Supervisor_(.*)-if00$/\1/')
fi
PORT=/dev/serial/by-id/usb-S3XY-BMS_Supervisor_${SUPERVISOR_SERIAL}-if00
echo "Supervisor: $SUPERVISOR_SERIAL"

if [ $BACKUP -eq 1 ]; then
    backup=~/supervisor_backup_$(date +%F_%H%M%S).uf2
    echo "Saving the running firmware to $backup..."
    picotool save -f --ser "$SUPERVISOR_SERIAL" "$backup"
    # save -f returns it to the application; wait for it to come back
    for _ in $(seq 30); do [ -e "$PORT" ] && break; sleep 0.5; done
fi

echo "Flashing..."
picotool load "$UF2" -f --ser "$SUPERVISOR_SERIAL"

echo "Waiting for it to come back on USB..."
for _ in $(seq 30); do [ -e "$PORT" ] && break; sleep 0.5; done
[ -e "$PORT" ] || die "the supervisor didn't come back on USB at $PORT"

# The firmware waits 4s after boot before starting; then ask for a status
stty -F "$PORT" sane nl -echo
timeout 8 cat "$PORT" &
reader=$!
sleep 6
echo s > "$PORT"
wait $reader || true
