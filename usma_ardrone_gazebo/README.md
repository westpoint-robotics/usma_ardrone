# Launching an experiment
## 1) launch the simulator, world, and robot descriptions
	roslaunch usma_ardrone_gazebo spawn_robots.launch 

## 2a) launch the drone and face tracking nodes
	roslaunch usma_ardrone_gazebo gazebo_track_face.launch 

## 2b) launch the mpc controller
	roslaunch usma_mpc denmpc_params.launch 

## 3) launch the uav
	roslaunch usma_mpc liftoff.launch