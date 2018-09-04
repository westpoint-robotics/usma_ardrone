Setting up the repo
=======
---

	git init
	git remote add gh git@github.com:westpoint-robotics/usma_ardrone.git

-- Under Construcition

required repos
=======
---

	sudo apt-get install -y ros-kinetic-ardrone-autonomy

might also need "hector_gazebo_plugins"

connect the ardrone to eecsds3
=======
---

connect to the ssid "ardrone2_<######>", in terminal type 
	>> roscd usma_ardrone && cd ../wpa_support 
	>> script/install
	>> script/connect_belkin
connect to create_belkin ssid
	>> ping 192.168.2.25
if the ping returns a response time	
	>> roslaunch optitrack_controller belkin.launch	
	>> roslaunch optitrack_controller vrpn.launch	


Finding a MAC address
=======
---
# telnet 192.168.1.1
# ifconfig

ardrone2_234879
90:03:B7:38:0D:72

ardrone2_065412
90:03:B7:31:18:D5  