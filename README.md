Setting up the repo
=======

	git init
	git remote add gh git@github.com:westpoint-robotics/usma_ardrone.git


---
Required repos:
=======

	sudo apt-get install -y ros-kinetic-ardrone-autonomy

might also need "hector_gazebo_plugins"

---
Set AR.Drone to connect to router:
=======

using a laptop or PC, connect to the ssid "ardrone2_<######>", in terminal type 

	>> roscd usma_ardrone && cd ../wpa_support 
	>> script/install
	>> script/connect_linksys

not connect the laptop to the linksys router network ARDRONE250024ghz

	>> ping 192.168.2.25

if the ping returns a response time, the the optitrack pc is connected to the same network


	>> roslaunch optitrack_controller belkin.launch	
	>> roslaunch optitrack_controller vrpn.launch	

--



---
Connect to the AR.Drone directly
=======
If you are directly connected to the ardrone netwrok, the following launch file will launch the ardrone controller
	roslaunch optitrack_controller ardrone_direct.launch


---
Launch the face_tracker when directly connected to the ardrone
=======
roslaunch optitrack_controller ardrone_direct.launch
roslaunch optitrack_controller track_face.launch


---
Connect to ARDrone using linksys router and running the whole face tracking pipeline
=======
roslaunch optitrack_controller track_face.launch network:=linksys
roslaunch optitrack_controller track_face.launch network:=belkin
roslaunch optitrack_controller track_face.launch network:=EECSDS3

---
Finding a MAC address
=======
# telnet 192.168.1.1
# ifconfig

ardrone2_234879
90:03:B7:38:0D:72

ardrone2_065412
90:03:B7:31:18:D5  
