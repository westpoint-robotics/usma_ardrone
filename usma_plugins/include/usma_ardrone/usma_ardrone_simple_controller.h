//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

/*
* quadrotor motion controller:
*
* This software is a motion control gazebo plugin for the Ardrone simulator
*
* change:
* 1. Noise is add to the callback function: VelocityCallback
* 2. Create a subscriber for rostopic /ardrone/navdata
* 3. An additional force and torque calculation is added base on the robot state information in /ardrone/navdata 
*
* Created on: Oct 22, 2012
* Author: Hongrong huang
*
*
*/

#ifndef VICON_PLUGINS_ARDRONE_SIMPLE_CONTROLLER_H
#define VICON_PLUGINS_ARDRONE_SIMPLE_CONTROLLER_H

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ardrone_autonomy/Navdata.h>

// msgs files
#include <usma_plugins/pid_controller.h>
#include <usma_plugins/pid_controllers.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#define UNKNOWN_MODEL       0
#define INITIALIZE_MODEL    1
#define LANDED_MODEL        2
#define FLYING_MODEL        3
#define HOVERING_MODEL      4
#define TESTING_MODEL       5
#define TAKINGOFF_MODEL     6
#define TO_FIX_POINT_MODEL  7
#define LANDING_MODEL       8
#define LOOPING_MODEL       9

namespace gazebo
{

class ARDroneSimpleController : public ModelPlugin
{
public:
  ARDroneSimpleController();
  virtual ~ARDroneSimpleController();

  void publishViconBaseTF();
    // vicon base frame
    std::string base_link_name_;
    std::string mocap_body_frame_;
    std::string mocap_origin_frame_;

    physics::ModelPtr vicon_base;
    physics::ModelPtr model;

    /// ROS pose publisher
    std::string mocap_pose_topic_;
    ros::Publisher mocap_pose_pub_;
    geometry_msgs::Pose mocap_pose_msg_;
    double publishedTime;


  std::string tf_prefix_;
  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

    
protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;
  physics::LinkPtr linkByName;
  math::Pose pose;
  math::Pose poseByName;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber state_subscriber_;

  bool publish_pid;
    ros::Publisher pids_pub;
    usma_plugins::pid_controllers pids_msg;
    std::string PIDRootTopic;

  // extra robot navi info subscriber
  std::string navdata_topic_;
  ros::Subscriber navdata_subscriber_;
  unsigned int navi_state;
  //***********************************
  
  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  geometry_msgs::Twist velocity_command_;
  // callback functions for subscribers
  void VelocityCallback(const geometry_msgs::TwistConstPtr&);
  void ImuCallback(const sensor_msgs::ImuConstPtr&);
  void StateCallback(const nav_msgs::OdometryConstPtr&);
  void NavdataCallback(const ardrone_autonomy::NavdataConstPtr& msg);

  ros::Time state_stamp, pub_stamp;

  math::Vector3 euler, velocity, acceleration, angular_velocity;

  std::string link_name_;
  std::string namespace_;
  std::string velocity_topic_;
  std::string imu_topic_;
  std::string state_topic_;
  double max_force_;
  double motion_small_noise_;
  double motion_drift_noise_;
  double motion_drift_noise_time_;
  double drift_noise_x_;
  double drift_noise_y_;
  double drift_noise_z_;
  double drift_noise_yaw_;


  class PIDController {
  public:
    PIDController();
    virtual ~PIDController();
    virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = "");

    double gain_p;
    double gain_i;
    double gain_d;
    double time_constant;
    double limit;

    double input;
    double dinput;
    double output;
    double p, i, d;

    usma_plugins::pid_controller pid_msg;
    std::string name;

    double update(double input, double x, double dx, double dt);
    void reset();
    void publishPID(double pub_time);
  };

  struct Controllers {
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
  } controllers_;

  math::Vector3 inertia;
  double mass;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif // VICON_PLUGINS_ARDRONE_SIMPLE_CONTROLLER_H
