New users start here: 
=======
https://github.com/DentOpt/denmpc

---
Install:
=======
	mkdir -p ~/ros/src/denmpc && cd ~/ros/src/denmpc 
	git init
	git remote add gh git@github.com:benjaminabruzzo/denmpc.git
	git pull gh master
	git fetch gh
	git checkout denmpc_ardrone_gazebo
	cd ~/ros
	catkin build denmpc

This branch also depends upon a separate repo: (I haven't tested just denmpc and usma_ardrone on a standalone PC)

	mkdir -p ~/ros/src/usma_ardrone && cd ~/ros/src/usma_ardrone
	git init
	git remote add gh git@github.com:westpoint-robotics/usma_ardrone.git
	git pull gh master
	cd ~/ros
	catkin build usma_ardrone

---
To run:
=======

Launch gazebo first:

	roslaunch usma_mpc gazebo.launch 

Signal the ardrone to takeoff:

	roslaunch usma_mpc liftoff.launch 

Launch denmpc:

	roslaunch usma_mpc ardrone_denmpc.launch 



---
Relevant source material:
=======
http://orbilu.uni.lu/bitstream/10993/28640/1/DENTLER_jan_MSC16_corrected.pdf