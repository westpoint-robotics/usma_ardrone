# update Autocorrect script directly


DRONEIP=${1:-"192.168.1.1"}
curl -T ARAutoConnect.sh "ftp://$DRONEIP"
# curl -T AutoConnectTest.sh "ftp://$DRONEIP"
curl -T sandbox.sh "ftp://$DRONEIP"


sleep 1

{( sleep 1; echo "
	mv /data/video/ARAutoConnect.sh /home/default/ARAutoConnect
	mv /data/video/sandbox.sh /home/default/ARAutoConnect
	chmod +x /home/default/ARAutoConnect/ARAutoConnect.sh
	chmod +x /home/default/ARAutoConnect/sandbox.sh
";) | telnet $DRONEIP > /dev/null; } | sleep 1 && echo "ARAutoConnect.sh Updated"