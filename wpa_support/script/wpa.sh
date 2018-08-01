#!/bin/sh

# Append
# /home/default/wpa.sh &
# to /bin/wifi_setup.sh

# The desired IP address for the drone
ADDRESS="192.168.2.25"

# WPA credentials for the new network
ESSID="create_belkin"
PASSWORD="hast2017"

# touch /etc/wpa_supplicant.conf
# rm /etc/wpa_supplicant.conf
# wpa_passphrase $ESSID $PASSWORD > /etc/wpa_supplicant.conf

# shorten this when everything is confirmed working
# sleep 1

# { ifconfig ath0 $ADDRESS; iwconfig ath0 essid '$ESSID' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf; } &
