	git init
	git remote add gh git@github.com:westpoint-robotics/usma_ardrone.git

-- Under Construcition

required repos:
sudo apt-get install -y ros-kinetic-ardrone-autonomy


-- connect the ardrone to eecsds3
--- connect to the ssid "ardrone2_234879"
--- in terminal type 
	>> roscd usma_ardrone/wpa_support
	>> sh ./script/install
	>> sh ./script/connect "create_belkin" -p "hast2017" -a auto -d 192.168.1.1
	>> sh ./script/connect "eecsds" -p "accessgranted" -a auto -d 192.168.1.1
--- close this terminal
--- connect to eecsds3
	>> roslaunch optitrack_controller ardrone.launch	


