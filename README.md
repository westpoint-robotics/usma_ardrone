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
Ar.Drone Face Tracking demo with optitrack feedback
=======

Set AR.Drone to connect to router, using a laptop or PC, connect to the ssid "ardrone2_<######>", in terminal type 

	>> roscd usma_ardrone && cd ../wpa_support 
	>> script/install_linksys
	>> script/connect_linksys

"install_linksys" needs only to be called once, but "connect_linksys" must be called each time the drone is powered down, such as when changing the battery. Now connect the laptop to the linksys router network ARDRONE250024ghz and test whether the ardrone is connected to the same network:

	>> ping 192.168.0.25

if the ping returns a response time, then the ardrone is connected to the same network on the correct ip address

---

Assuming the optitrack software and cameras are booted and running, to launch the demo, first launch the vrpn service. This will stream the optitrack pose data as a ros message. (note this assumes the Ethernet cable for the optitrack pc has been switched from EECSDS3 to the linksys router, it has not been tested on EECSDS3)

	>> roslaunch optitrack_controller vrpn.launch

To run the face tracking demo, the following command file will launch the control and tracking nodes for the ardrone

	>> roslaunch optitrack_controller track_face.launch network:=linksys

Finally to have the drone takeoff :

	>> rostopic pub -1 /ardrone/takeoff std_msgs/Empty

and to land;

	>> rostopic pub -1 /ardrone/land std_msgs/Empty

If there is an issue during takeoff, or if you need to do a hard abort for some reason, you may need to reset the drone before taking off a second time:
	
	>> rostopic pub -1 /ardrone/reset std_msgs/Empty

---
Connect to the AR.Drone directly
=======
If you are directly connected to the ardrone network, the following launch file will launch the ardrone controller
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
