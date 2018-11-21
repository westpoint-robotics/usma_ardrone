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

See readme under optitrack_controller


---
testing mpc:
=======
see readme under usma_mpc


---
Finding a MAC address
=======
	>> telnet 192.168.1.1
	>> ifconfig

ardrone2_234879  ::  90:03:B7:38:0D:72 <br />
ardrone2_065412  ::  90:03:B7:31:18:D5 <br />
ardrone2_049677  ::  90:03:B7:E8:3C:D8 <br />
