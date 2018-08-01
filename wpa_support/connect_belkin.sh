#!/bin/bash
roscd optitrack_controller && cd ../wpa_support && sh ./script/install


# sh /home/benjamin/ros/src/usma_ardrone/wpa_support/script/connect "create_belkin" -p "hast2017" -a 192.168.2.25 -d 192.168.1.1
# sh ./script/connect "eecsds" -p "accessgranted" -a auto -d 192.168.1.1



#

# echo wpa_passphrase 'create_belkin' 'hast2017' > /etc/wpa_supplicant.conf


curl -T wpa_supplicant.conf "ftp://192.168.1.1/usr/wpa_supplicant.conf"
