git clone git@bitbucket.org:benjaminabruzzo/hast-ros-turtlebot.git

# Git paths
	git remote add bb git@bitbucket.org:benjaminabruzzo/hast-ros-turtlebot.git
	git remote add pluto git@pluto.local:benjamin/hast-ros-turtlebot.git
	
	<!-- git remote kobuki https://github.com/[user]/[repo].git -->
	git remote add mars git@mars.local:hast/ros-turtlebot.git
	git remote add bb git@bitbucket.org:benjaminabruzzo/hast-ros-turtlebot.git
	git remote add venus git@venus.local:benjamin/hast-ros.git
	

# Launching an experiment
## 1) launch the simulator, world, and robot descriptions
	roslaunch hast_gazebo spawn_robots.launch world:=empty
	<arg name="world"   default="empty"/>    
	world:=block ::> load world with obstruction in it

## 2) launch the recorder and KF files with dates
	(roscd hast && . scripts/makedirs 20180415 1 5)
	roslaunch hast_gazebo hast.launch date:=20180415 run:=001
### 2a) flags:
	saveimages:=true 	:: saves images
	showcircles:=true 	:: shows left and right images with "found" circles

## 3) launch the experiment
	roslaunch hast experiment.launch 
### 3a) flags:
	abcde:=true 	:: UAV flys through waypoints A-E
	<arg name="arc" 		default="false" />
	<arg name="pull" 		default="false" />
	<arg name="picket" 		default="false" />
	<arg name="abcde" 		default="false" />
	<arg name="step" 		default="false" />
	<arg name="ugvfore" 	default="false" />
	<arg name="ugvauto" 	default="false" />
	<arg name="iflift" 		default="false" />
	<arg name="aprillog" 	default="false" />
	<arg name="ugvcomplex" 	default="false" />
	<arg name="action" 		default="false" />
	<arg name="wait" 		default="15" />



## ZZZ) TRENDNET_CREATE::turtlebot_create
