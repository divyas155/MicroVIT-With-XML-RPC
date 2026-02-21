#!/bin/bash
# Disable USB autosuspend for Logitech camera (046d:0825).
# dmesg often shows "usb_suspend_both" before camera disconnects.
# Run on Nano before starting bringup: ./disable_usb_autosuspend.sh

VENDOR="046d"
PRODUCT="0825"

for dev in /sys/bus/usb/devices/*/idVendor; do
    [ -f "$dev" ] || continue
    ven=$(cat "$dev" 2>/dev/null)
    prod=$(cat "${dev%Vendor}Product" 2>/dev/null)
    if [ "$ven" = "$VENDOR" ] && [ "$prod" = "$PRODUCT" ]; then
        dir=$(dirname "$dev")
        if [ -f "$dir/power/autosuspend" ]; then
            echo -1 | sudo tee "$dir/power/autosuspend" > /dev/null
            echo "Disabled USB autosuspend for Logitech camera: $dir"
        fi
    fi
done
