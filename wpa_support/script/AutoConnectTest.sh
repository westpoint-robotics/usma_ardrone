#!/bin/sh

# This file is part of ARAutoConnect.
#
# ARAutoConnect is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# ARAutoConnect is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ARAutoConnect.  If not, see <http://www.gnu.org/licenses/>.


INSTALL_PATH=/home/default/ARAutoConnect

ESSID_PATH=$INSTALL_PATH/essid
IP_PATH=$INSTALL_PATH/ip
NETMASK_PATH=$INSTALL_PATH/netmask

LOG=$INSTALL_PATH/logbox

ESSID=rezo
IP=192.168.1.1
NETMASK=255.255.0.0

BASE_ADDRESS=192.168.1.
LAST_NUMBER="2 3 4 5"
CONNECTED=0
COUNT=0

#retrieving ad-hoc ssid to restore the network
DRONESSID=`grep ssid_single_player /data/config.ini | awk -F "=" '{print $2}'`

# Removing leading and trailing spaces
DRONESSID=`echo $DRONESSID`

if [ -n "$DRONESSID" ]
then
        echo "found drone ssid =$DRONESSID"
else
        #default SSID.
        DRONESSID=ardrone_wifi
fi

#retrieving managed mode configuration
if [ -s $ESSID_PATH ] ; then
        ESSID=`cat $ESSID_PATH`
fi

if [ -s $IP_PATH ] ; then
        IP=`cat $IP_PATH`
fi

if [ -s $NETMASK_PATH ] ; then
        NETMASK=`cat $NETMASK_PATH`
fi

echo "Network configuration" > $LOG
echo "Drone SSID                : $DRONESSID" >> $LOG
echo "Access Point ESSID        : $ESSID" >> $LOG
echo "Infrastructure IP address : $IP" >> $LOG
echo "Infrastructure Netmask    : $NETMASK" >>$LOG

# CONFIG=`iwconfig ath0`
# echo " " >> $LOG
# echo $CONFIG >> log
# echo " " >> $LOG

MASTERn=0
MASTERLIMIT=3
ADHOCn=1
echo "Init Mode: MASTERn = $MASTERn, ADHOCn = $ADHOCn" >> $LOG
echo " --- " >> $LOG

while [ "$MASTERn" -le 20 ]
do


    # CONFIG=`iwconfig ath0`

	echo "$MASTERn >?= $MASTERLIMIT"

    if [ "$MASTERn" -ge "$MASTERLIMIT" ] ; then
    	echo "$MASTERn >= $MASTERLIMIT"
        echo "$MASTERn >= $MASTERLIMIT" >> $LOG
        # sleep 1
    fi

	MASTERn=`expr "$MASTERn" + 1`


done