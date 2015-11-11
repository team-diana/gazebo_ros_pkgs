/*
 * Copyright (c) 2014 Team Diana
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_joint_commander.h
 *
 * \brief A configurable plugin that controls one or more joint.
 *
 * \author  Vincenzo Comito <clynamen@gmail.com>
 */

#ifndef GAZEBO_ROS_JOINT_COMMANDER_H_
#define GAZEBO_ROS_JOINT_COMMANDER_H_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <memory>

#include <ros/ros.h>

namespace gazebo {

  class Joint;
  class Entity;

  class JointSet {
    public:
      JointSet(bool instantaneous, std::string name, std::vector<physics::JointPtr> joints, std::vector<double> jointSpeeds) :
      instantaneous(instantaneous),
      jointSpeeds(jointSpeeds),
      name(name),
      joints(joints)
      {
        targetPositions.resize(joints.size());
        std::fill(targetPositions.begin(), targetPositions.end(), 0);
      }

      int GetDof() { return joints.size(); }
      const std::string& GetName() { return name; }
      physics::JointPtr GetJoint(int index) { return joints[index]; }
      const std::vector<physics::JointPtr>& GetJoints() { return joints; }
      std::vector<double> jointSpeeds;
      std::vector<double> targetPositions;
      bool instantaneous;

    private:
      std::string name;
      std::vector<physics::JointPtr> joints;
  };

  class ControlledJointSet {
    public:
    ControlledJointSet(std::shared_ptr<JointSet> joint_set)
    : joint_set(joint_set) {}

    ~ControlledJointSet() {
      joint_set;
    }

    void update(double time_delta);
    void publish(double time);
    void cmdPositionCallback(const sensor_msgs::JointState::ConstPtr& cmd_msg);

    std::shared_ptr<JointSet> joint_set;
    ros::Subscriber rosSubscriber;
    ros::Publisher rosPublisher;
  };

  class GazeboRosJointCommander : public ModelPlugin {

    public:
	  GazeboRosJointCommander();
    ~GazeboRosJointCommander();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    std::shared_ptr<JointSet> LoadJointSet(sdf::ElementPtr _sdf, int index);

    void virtual UpdateChild();

    static const std::string PLUGIN_NAME;

    protected:
      void Shutdown();

    private:

      physics::WorldPtr world;
      physics::ModelPtr parent;

      std::vector<std::unique_ptr<ControlledJointSet>> controlled_joints;
      physics::JointPtr joints[4];

      // ROS STUFF
      std::unique_ptr<ros::NodeHandle> rosnode_;
      bool broadcast_tf_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      event::ConnectionPtr updateConnection;
      void QueueThread();


      double x_;
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;
      common::Time last_publish_time_;

  };

}


#endif

