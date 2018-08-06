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
* Ardrone motion controller:
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
#include <usma_ardrone/usma_ardrone_simple_controller.h>
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"

// msg headers
// #include <usma_plugins/pid_controller.h>
#include <usma_plugins/pid_controllers.h>

#include <cmath>
#include <stdlib.h>

namespace gazebo {

  ARDroneSimpleController::ARDroneSimpleController()
  {
    navi_state = 0;
  }

  // Destructor //////////////////////////////////////////////////////////////////
    ARDroneSimpleController::~ARDroneSimpleController()
    {
      event::Events::DisconnectWorldUpdateBegin(updateConnection);

      node_handle_->shutdown();
      delete node_handle_;
    }

  // Load the controller /////////////////////////////////////////////////////////
    void ARDroneSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      world = _model->GetWorld();

      // load parameters
      if (!_sdf->HasElement("robotNamespace"))
        namespace_.clear();
      else
        namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString() + "/";

      if (!_sdf->HasElement("topicName"))
        velocity_topic_ = "cmd_vel";
      else
        velocity_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

      if (!_sdf->HasElement("navdataTopic"))
        navdata_topic_ = "/ardrone/navdata";
      else
        navdata_topic_ = _sdf->GetElement("navdataTopic")->GetValue()->GetAsString();

      if (!_sdf->HasElement("imuTopic"))
        imu_topic_.clear();
      else
        imu_topic_ = _sdf->GetElement("imuTopic")->GetValue()->GetAsString();

      if (!_sdf->HasElement("stateTopic"))
        state_topic_.clear();
      else
        state_topic_ = _sdf->GetElement("stateTopic")->GetValue()->GetAsString();

      if (!_sdf->HasElement("bodyName"))
      {
        link = _model->GetLink();
        link_name_ = link->GetName();
      }
      else {
        link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
        link = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_name_));
      }

        linkByName = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_name_));


      if (!link)
      {
        ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
        return;
      }

      if (!_sdf->HasElement("maxForce"))
        max_force_ = -1;
      else
        _sdf->GetElement("maxForce")->GetValue()->Get(max_force_);


      if (!_sdf->HasElement("motionSmallNoise"))
        motion_small_noise_ = 0;
      else
        _sdf->GetElement("motionSmallNoise")->GetValue()->Get(motion_small_noise_);

      if (!_sdf->HasElement("motionDriftNoise"))
        motion_drift_noise_ = 0;
      else
        _sdf->GetElement("motionDriftNoise")->GetValue()->Get(motion_drift_noise_);

      if (!_sdf->HasElement("motionDriftNoiseTime"))
        motion_drift_noise_time_ = 1.0;
      else
        _sdf->GetElement("motionDriftNoiseTime")->GetValue()->Get(motion_drift_noise_time_);

      if (!_sdf->HasElement("drift_noise_x"))
        drift_noise_x_ = 0.0;
      else
        _sdf->GetElement("drift_noise_x")->GetValue()->Get(drift_noise_x_);

      if (!_sdf->HasElement("drift_noise_y"))
        drift_noise_y_ = 0.0;
      else
        _sdf->GetElement("drift_noise_y")->GetValue()->Get(drift_noise_y_);

      if (!_sdf->HasElement("drift_noise_z"))
        drift_noise_z_ = 0.0;
      else
        _sdf->GetElement("drift_noise_z")->GetValue()->Get(drift_noise_z_);

      if (!_sdf->HasElement("drift_noise_yaw"))
        drift_noise_yaw_ = 0.0;
      else
        _sdf->GetElement("drift_noise_yaw")->GetValue()->Get(drift_noise_yaw_);


      if (!_sdf->HasElement("publishPID"))
        publish_pid = false;
      else
        _sdf->GetElement("publishPID")->GetValue()->Get(publish_pid);

      if (!_sdf->HasElement("PIDRootTopic"))
        PIDRootTopic = "/hast/uav/pid/";
      else
        _sdf->GetElement("PIDRootTopic")->GetValue()->Get(PIDRootTopic);

      /* Vicon config */
        transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

      // typedef boost::shared_ptr<Model> ModelPtr
        vicon_base = _model;
        base_link_name_ = vicon_base->GetName();
        ROS_WARN_NAMED("ardrone_simple_controller", "base_link_name_ := %s", base_link_name_.c_str());

        link_red_name_ = _sdf->GetElement("red_led_link")->GetValue()->GetAsString();
        link_red = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_red_name_)); 
        ROS_WARN_NAMED("ardrone_simple_controller", "link_red_name_   := %s", link_red_name_.c_str());
        
        link_blue_name_ = _sdf->GetElement("blue_led_link")->GetValue()->GetAsString();
        link_blue = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_blue_name_)); 
        ROS_WARN_NAMED("ardrone_simple_controller", "link_blue_name_  := %s", link_blue_name_.c_str());
        
        link_green_name_ = _sdf->GetElement("green_led_link")->GetValue()->GetAsString();
        link_green = boost::dynamic_pointer_cast<physics::Link>(world->GetEntity(link_green_name_)); 
        ROS_WARN_NAMED("ardrone_simple_controller", "link_green_name_ := %s", link_green_name_.c_str());
      /* Vicon config */

      // Get inertia and mass of quadrotor body
      inertia = link->GetInertial()->GetPrincipalMoments();
      mass = link->GetInertial()->GetMass();

      node_handle_ = new ros::NodeHandle(namespace_);

      controllers_.roll.Load(_sdf, "rollpitch");        
        controllers_.roll.name = "roll";
      controllers_.pitch.Load(_sdf, "rollpitch");
        controllers_.pitch.name = "pitch";
      controllers_.yaw.Load(_sdf, "yaw");
        controllers_.yaw.name = "yaw";
      controllers_.velocity_x.Load(_sdf, "velocityXY");
        controllers_.velocity_x.name = "velocity_x";
      controllers_.velocity_y.Load(_sdf, "velocityXY");
        controllers_.velocity_y.name = "velocity_y";
      controllers_.velocity_z.Load(_sdf, "velocityZ");
        controllers_.velocity_z.name = "velocity_z";

      if(publish_pid)
        {
          ROS_INFO("ARDroneSimpleController:: Publish PIDs");
          pids_pub = node_handle_->advertise<usma_plugins::pid_controllers>(PIDRootTopic.c_str(), 30);
          pids_msg.stamp = ros::Time::now().toSec(); // # time of measurement
          pids_msg.id = 0; //     # serial number of message
        } else {
          ROS_INFO("ARDroneSimpleController:: DO NOT Publish PIDs");
        }

      // subscribe command: velocity control command
      if (!velocity_topic_.empty())
      {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
          velocity_topic_, 1,
          boost::bind(&ARDroneSimpleController::VelocityCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
        velocity_subscriber_ = node_handle_->subscribe(ops);
      }

      // subscribe command: navigation data
      if (!navdata_topic_.empty())
      {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<ardrone_autonomy::Navdata>(
          navdata_topic_, 1,
          boost::bind(&ARDroneSimpleController::NavdataCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
        navdata_subscriber_ = node_handle_->subscribe(ops);
      }
        //m_navdataPub = node_handle_->advertise< ardrone_autonomy::Navdata >( "/ardrone/navdata", 10 );


      // subscribe imu
      if (!imu_topic_.empty())
      {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
          imu_topic_, 1,
          boost::bind(&ARDroneSimpleController::ImuCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
        imu_subscriber_ = node_handle_->subscribe(ops);

        ROS_INFO_NAMED("ardrone_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
      }

      // subscribe state
      if (!state_topic_.empty())
      {
        ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
          state_topic_, 1,
          boost::bind(&ARDroneSimpleController::StateCallback, this, _1),
          ros::VoidPtr(), &callback_queue_);
        state_subscriber_ = node_handle_->subscribe(ops);

        ROS_INFO_NAMED("ardrone_simple_controller", "Using state information on topic %s as source of state information.", state_topic_.c_str());
      }

      // callback_queue_thread_ = boost::thread( boost::bind( &ARDroneSimpleController::CallbackQueueThread,this ) );


      Reset();

      // New Mechanism for Updating every World Cycle
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ARDroneSimpleController::Update, this));
    }

  // Callbacks ///////////////////////////////////////////////////////////////////

    void ARDroneSimpleController::publishViconBaseTF()
    {
        ros::Time current_time = ros::Time::now();
        math::Pose base_pose = vicon_base->GetWorldPose();

        red_pose = link_red->GetWorldPose();
        blue_pose = link_blue->GetWorldPose();
        green_pose = link_green->GetWorldPose();

        tf::Quaternion qt ( base_pose.rot.x, base_pose.rot.y, base_pose.rot.z, base_pose.rot.w);
        tf::Vector3 vt ( base_pose.pos.x, base_pose.pos.y, base_pose.pos.z);
        tf::Transform baseTF ( qt, vt );
        transform_broadcaster_->sendTransform (tf::StampedTransform ( baseTF, current_time, "/vicon/origin", "/vicon/uav" ) );
        transform_broadcaster_->sendTransform (tf::StampedTransform ( baseTF, current_time, "/vicon/origin", "/vicon/uav/base_footprint" ) );

        tf::Quaternion red_qt ( red_pose.rot.x, red_pose.rot.y, red_pose.rot.z, red_pose.rot.w);
        tf::Vector3 red_vt ( red_pose.pos.x, red_pose.pos.y, red_pose.pos.z);
        tf::Transform red_baseTF ( red_qt, red_vt );
        transform_broadcaster_->sendTransform (tf::StampedTransform ( red_baseTF, current_time, "/vicon/origin", "/vicon/uav/ardrone_red_led" ) );

        tf::Quaternion blue_qt ( blue_pose.rot.x, blue_pose.rot.y, blue_pose.rot.z, blue_pose.rot.w);
        tf::Vector3 blue_vt ( blue_pose.pos.x, blue_pose.pos.y, blue_pose.pos.z);
        tf::Transform blue_baseTF ( blue_qt, blue_vt );
        transform_broadcaster_->sendTransform (tf::StampedTransform ( blue_baseTF, current_time, "/vicon/origin", "/vicon/uav/ardrone_blue_led" ) );

        tf::Quaternion green_qt ( green_pose.rot.x, green_pose.rot.y, green_pose.rot.z, green_pose.rot.w);
        tf::Vector3 green_vt ( green_pose.pos.x, green_pose.pos.y, green_pose.pos.z);
        tf::Transform green_baseTF ( green_qt, green_vt );
        transform_broadcaster_->sendTransform (tf::StampedTransform ( green_baseTF, current_time, "/vicon/origin", "/vicon/uav/ardrone_green_led" ) );

    }



    void ARDroneSimpleController::VelocityCallback(const geometry_msgs::TwistConstPtr& velocity)
    {
      velocity_command_ = *velocity;


      static common::Time last_sim_time = world->GetSimTime();
      static double time_counter_for_drift_noise = 0;
      static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
      // Get simulator time
      common::Time cur_sim_time = world->GetSimTime();
      double dt = (cur_sim_time - last_sim_time).Double();
      // save last time stamp
      last_sim_time = cur_sim_time;

      // generate noise
      if(time_counter_for_drift_noise > motion_drift_noise_time_)
      {
        // drift_noise[0] = 2*motion_drift_noise_*(drand48()-0.5);
        // drift_noise[1] = 2*motion_drift_noise_*(drand48()-0.5);
        // drift_noise[2] = 2*motion_drift_noise_*(drand48()-0.5);
        // drift_noise[3] = 2*motion_drift_noise_*(drand48()-0.5);
        time_counter_for_drift_noise = 0.0;

        
        drift_noise[0] = drift_noise_x_*(drand48()-0.5);
        drift_noise[1] = drift_noise_y_*(drand48()-0.5);
        drift_noise[2] = drift_noise_z_*(drand48()-0.5);
        drift_noise[3] = drift_noise_yaw_*(drand48()-0.5);
      }
      time_counter_for_drift_noise += dt;

      velocity_command_.linear.x += drift_noise[0] + 2*motion_small_noise_*(drand48()-0.5);
      velocity_command_.linear.y += drift_noise[1] + 2*motion_small_noise_*(drand48()-0.5);
      velocity_command_.linear.z += drift_noise[2] + 2*motion_small_noise_*(drand48()-0.5);
      velocity_command_.angular.z += drift_noise[3] + 2*motion_small_noise_*(drand48()-0.5);
      //  velocity_command_.angular.z *= 2;
    }

    void ARDroneSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
    {
      pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
      euler = pose.rot.GetAsEuler();
      angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
    }

    void ARDroneSimpleController::StateCallback(const nav_msgs::OdometryConstPtr& state)
    {
      math::Vector3 velocity1(velocity);

      if (imu_topic_.empty()) {
        pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
        pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
        euler = pose.rot.GetAsEuler();
        angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
      }

      velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

      // calculate acceleration
      double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
      state_stamp = state->header.stamp;
      if (dt > 0.0) {
        acceleration = (velocity - velocity1) / dt;
      } else {
        acceleration.Set();
      }
    }

  // Update the controller ///////////////////////////////////////////////////////
    void ARDroneSimpleController::Update()
    {
      publishViconBaseTF();
      math::Vector3 force, torque;

      // Get new commands/state
      callback_queue_.callAvailable();

      // Get simulator time
      common::Time sim_time = world->GetSimTime();
      double dt = (sim_time - last_time).Double();
      if (dt == 0.0) return;

      // Get Pose/Orientation from Gazebo (if no state subscriber is active)
      if (imu_topic_.empty()) {
        pose = link->GetWorldPose();
        angular_velocity = link->GetWorldAngularVel();
        euler = pose.rot.GetAsEuler();
      }
      if (state_topic_.empty()) {
        acceleration = (link->GetWorldLinearVel() - velocity) / dt;
        velocity = link->GetWorldLinearVel();
      }

      //  static Time lastDebug;
        //  if ((world->GetSimTime() - lastDebug).Double() > 0.5) {
        //    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Velocity:         gazebo = [" << link->GetWorldLinearVel()   << "], state = [" << velocity << "]");
        //    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Acceleration:     gazebo = [" << link->GetWorldLinearAccel() << "], state = [" << acceleration << "]");
        //    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Angular Velocity: gazebo = [" << link->GetWorldAngularVel() << "], state = [" << angular_velocity << "]");
        //    lastDebug = world->GetSimTime();
        //  }

      // Get gravity
      math::Vector3 gravity_body = pose.rot.RotateVector(world->GetPhysicsEngine()->GetGravity());
      double gravity = gravity_body.GetLength();
      double load_factor = gravity * gravity / world->GetPhysicsEngine()->GetGravity().Dot(gravity_body);  // Get gravity

      // Rotate vectors to coordinate frames relevant for control
      math::Quaternion heading_quaternion(cos(euler.z/2),0,0,sin(euler.z/2));
      math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
      math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
      math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);

      // update controllers
      force.Set(0.0, 0.0, 0.0);
      torque.Set(0.0, 0.0, 0.0);

      /////////// serial vel to pitch control
      double pitch_command =  controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
      double roll_command  = -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.y, acceleration_xy.y, dt) / gravity;
      torque.x = inertia.x *  controllers_.roll.update (roll_command,  euler.x, angular_velocity_body.x, dt);
      torque.y = inertia.y *  controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
      
      ///////////// direct pitch controll
      // torque.x = inertia.x *  controllers_.roll.update (velocity_command_.linear.x,  euler.x, angular_velocity_body.x, dt);
      // torque.y =-inertia.y *  controllers_.pitch.update(velocity_command_.linear.y, euler.y, angular_velocity_body.y, dt);
      torque.z = inertia.z *  controllers_.yaw.update(velocity_command_.angular.z, angular_velocity.z, 0, dt);
      force.z  = mass      * (controllers_.velocity_z.update(velocity_command_.linear.z,  velocity.z, acceleration.z, dt) + load_factor * gravity);
      if (max_force_ > 0.0 && force.z > max_force_) force.z = max_force_;
      if (force.z < 0.0) force.z = 0.0;

      // ROS_INFO("UAV Controller::force.[x y z] torque.[x y z] = [%4.4f %4.4f %4.4f][%4.4f %4.4f %4.4f]",  force.x,  force.y, force.z, torque.x, torque.y, torque.z);

      //  static double lastDebugOutput = 0.0;
      //  if (last_time.Double() - lastDebugOutput > 0.1) {
        //    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Velocity = [%g %g %g], Acceleration = [%g %g %g]", velocity.x, velocity.y, velocity.z, acceleration.x, acceleration.y, acceleration.z);
        //    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Command: linear = [%g %g %g], angular = [%g %g %g], roll/pitch = [%g %g]", velocity_command_.linear.x, velocity_command_.linear.y, velocity_command_.linear.z, velocity_command_.angular.x*180/M_PI, velocity_command_.angular.y*180/M_PI, velocity_command_.angular.z*180/M_PI, roll_command*180/M_PI, pitch_command*180/M_PI);
        //    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Mass: %g kg, Inertia: [%g %g %g], Load: %g g", mass, inertia.x, inertia.y, inertia.z, load_factor);
        //    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Force: [%g %g %g], Torque: [%g %g %g]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
        //    lastDebugOutput = last_time.Double();
      //  }

      // process robot state information
        if(navi_state == LANDED_MODEL)
        {

        }
        else if((navi_state == FLYING_MODEL)||(navi_state == TO_FIX_POINT_MODEL))
        {
          link->AddRelativeForce(force);
          link->AddRelativeTorque(torque);
        }
        else if(navi_state == TAKINGOFF_MODEL)
        {
          link->AddRelativeForce(force*1.5);
          link->AddRelativeTorque(torque*1.5);
        }
        else if(navi_state == LANDING_MODEL)
        {
          link->AddRelativeForce(force*0.8);
          link->AddRelativeTorque(torque*0.8);
        }


        if(publish_pid)
          {
            // ROS_INFO("ARDroneSimpleController:: Publish PIDs");
            controllers_.roll.publishPID(sim_time.Double());
            controllers_.pitch.publishPID(sim_time.Double());
            controllers_.yaw.publishPID(sim_time.Double());
            controllers_.velocity_x.publishPID(sim_time.Double());
            controllers_.velocity_y.publishPID(sim_time.Double());
            controllers_.velocity_z.publishPID(sim_time.Double());

            //clear last message
            pids_msg.controllers.clear();
            //add new data
            pids_msg.stamp = sim_time.Double();
            pids_msg.id += 1; 
            pids_msg.controllers.push_back(controllers_.roll.pid_msg);
            pids_msg.controllers.push_back(controllers_.pitch.pid_msg);
            pids_msg.controllers.push_back(controllers_.yaw.pid_msg);
            pids_msg.controllers.push_back(controllers_.velocity_x.pid_msg);
            pids_msg.controllers.push_back(controllers_.velocity_y.pid_msg);
            pids_msg.controllers.push_back(controllers_.velocity_z.pid_msg);
            //Publish array
            pids_pub.publish(pids_msg);

          } else {
            // ROS_INFO("ARDroneSimpleController:: DO NOT Publish PIDs");
          }

      // save last time stamp
      last_time = sim_time;
    }

  // Reset the controller ////////////////////////////////////////////////////////
    void ARDroneSimpleController::Reset()
    {
      controllers_.roll.reset();
      controllers_.pitch.reset();
      controllers_.yaw.reset();
      controllers_.velocity_x.reset();
      controllers_.velocity_y.reset();
      controllers_.velocity_z.reset();

      link->SetForce(math::Vector3(0,0,0));
      link->SetTorque(math::Vector3(0,0,0));

      // reset state
      pose.Reset();
      velocity.Set();
      angular_velocity.Set();
      acceleration.Set();
      euler.Set();
      state_stamp = ros::Time();
    }

  // PID controller implementation ///////////////////////////////////////////////
    ARDroneSimpleController::PIDController::PIDController()
    {
    }

    ARDroneSimpleController::PIDController::~PIDController()
    {
    }

    void ARDroneSimpleController::PIDController::Load(sdf::ElementPtr _sdf, const std::string& prefix)
    {
      gain_p = 0.0;
      gain_d = 0.0;
      gain_i = 0.0;
      time_constant = 0.0;
      limit = -1.0;
      // topic = "/pid/";

      if (!_sdf) return;
      // _sdf->PrintDescription(_sdf->GetName());
      if (_sdf->HasElement(prefix + "ProportionalGain")) _sdf->GetElement(prefix + "ProportionalGain")->GetValue()->Get(gain_p);
      if (_sdf->HasElement(prefix + "DifferentialGain")) _sdf->GetElement(prefix + "DifferentialGain")->GetValue()->Get(gain_d);
      if (_sdf->HasElement(prefix + "IntegralGain"))     _sdf->GetElement(prefix + "IntegralGain")->GetValue()->Get(gain_i);
      if (_sdf->HasElement(prefix + "TimeConstant"))     _sdf->GetElement(prefix + "TimeConstant")->GetValue()->Get(time_constant);
      if (_sdf->HasElement(prefix + "Limit"))            _sdf->GetElement(prefix + "Limit")->GetValue()->Get(limit);

      // if (_sdf->HasElement(prefix + "topic"))            
      //   {
      //     _sdf->GetElement(prefix + "topic")->GetValue()->Get(topic);
      //   }

      // pid_msg.stamp = ros::Time::now().toSec(); // # time of measurement
      // pid_msg.id = 1; //     # serial number of message

      pid_msg.errors.p = 0;
      pid_msg.errors.i = 0;
      pid_msg.errors.d = 0;
      pid_msg.gains.p = gain_p;
      pid_msg.gains.i = gain_i;
      pid_msg.gains.d = gain_d;

      pid_msg.input = 0;
      pid_msg.dinput = 0;
      pid_msg.time_constant = time_constant;
      pid_msg.limit = limit;
      pid_msg.output = 0;
    }


    double ARDroneSimpleController::PIDController::update(double new_input, double x, double dx, double dt)
    {
      // limit command
      if (limit > 0.0 && fabs(new_input) > limit) new_input = (new_input < 0 ? -1.0 : 1.0) * limit;

      // filter command
      if (dt + time_constant > 0.0) {
        dinput = (new_input - input) / (dt + time_constant);
        input  = (dt * new_input + time_constant * input) / (dt + time_constant);
      }

      // update proportional, differential and integral errors
      pid_msg.state = x;
      p = input - x;
      d = dinput - dx;
      i = i + dt * p;

      // update control output
      output = gain_p * p + gain_d * d + gain_i * i;

      return output;
    }

    void ARDroneSimpleController::PIDController::publishPID(double pub_time)
    {
      pid_msg.name = name;

      pid_msg.errors.p = p;
      pid_msg.errors.i = i;
      pid_msg.errors.d = d;
      pid_msg.gains.p = gain_p;
      pid_msg.gains.i = gain_i;
      pid_msg.gains.d = gain_d;

      pid_msg.input = input;
      pid_msg.dinput = dinput;
      pid_msg.time_constant = time_constant;
      pid_msg.limit = limit;
      pid_msg.output = output;

    }


    void ARDroneSimpleController::PIDController::reset()
    {
      input = dinput = 0;
      p = i = d = output = 0;
    }

    void ARDroneSimpleController::NavdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
    {
      navi_state = msg -> state;
    }

  // Register this plugin with the simulator /////////////////////////////////////
  GZ_REGISTER_MODEL_PLUGIN(ARDroneSimpleController)

} // namespace gazebo
