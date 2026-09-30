#!/bin/bash
# Restore normal desk-use settings after a field deployment:
# - stops (and disables) the field recorder, so it releases both cameras and
#   you can use stream_server.py / field_recorder.py manually
# - turns Wi-Fi, Bluetooth, and wired Ethernet back on
# - switches the boot target back to the desktop
#
# Reverses everything done by field_mode.sh.
set -e

echo "Stopping and disabling the field recorder..."
sudo systemctl disable --now field-recorder.service

echo "Re-enabling wired Ethernet..."
# field-mode-net.service's ExecStop brings eth0 back up, so disabling it
# (rather than a separate `ip link up` command) is the actual reversal.
sudo systemctl disable --now field-mode-net.service

echo "Letting NetworkManager manage eth0 again..."
NM_CONF="/etc/NetworkManager/conf.d/99-unmanage-eth0.conf"
if [ -f "$NM_CONF" ]; then
    sudo rm -f "$NM_CONF"
    sudo systemctl reload NetworkManager 2>/dev/null || true
    sudo nmcli device set eth0 managed yes 2>/dev/null || true
fi

echo "Unblocking Wi-Fi and Bluetooth radios..."
sudo rfkill unblock wifi
sudo rfkill unblock bluetooth

echo "Re-enabling the Bluetooth daemon..."
sudo systemctl enable --now bluetooth.service

echo "Switching the boot target back to desktop..."
# No-op with a harmless warning if this is a Lite (no-desktop) install.
sudo systemctl set-default graphical.target

echo
echo "Normal mode is set. Reboot to return to the desktop and Wi-Fi:"
echo "  sudo reboot"
