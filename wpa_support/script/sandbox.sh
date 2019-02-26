#!/usr/bin/env bash

ESSID="ARDRONE250024ghz"
PASSWORD="abruzzo2018"
ADDRESS="192.168.0.25"
DRONEIP="192.168.1.1"
IFCONFIG="ifconfig ath0 $ADDRESS"
DHCPC=""

echo "ESSID: $ESSID"
echo "PASSWORD: $PASSWORD"
echo "ADDRESS: $ADDRESS"
echo "DRONE_IP: $DRONEIP"


echo "{ $IFCONFIG; iwconfig ath0 essid '$ESSID' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf $DHCPC; }" 


{ $IFCONFIG; iwconfig ath0 essid '$ESSID' && wpa_supplicant -B -Dwext -iath0 -c/etc/wpa_supplicant.conf $DHCPC; }

