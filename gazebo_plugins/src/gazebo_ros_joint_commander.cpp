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
 * \file  gazebo_ros_joint_commander.cpp
 *
 * \brief A configurable plugin that controls one or more joint.
 *
 * \author Vincenzo Comito <clynamen@gmail.com>
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_joint_commander.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <team_diana_lib/logging/logging.h>

namespace gazebo {

  using namespace Td;

  enum {
    RIGHT_FRONT=0,
    LEFT_FRONT=1,
    RIGHT_REAR=2,
    LEFT_REAR=3,
  };

  const std::string GazeboRosJointCommander::PLUGIN_NAME = "GazeboRosJointCommander";

  GazeboRosJointCommander::GazeboRosJointCommander() : alive_(true) {}

  // Destructor
  GazeboRosJointCommander::~GazeboRosJointCommander() {
    ros_info(" destroying  ");
    delete rosnode_;
    for(auto controlled_join : controlled_joints) {
      delete controlled_join;
    }
  }

  JointSet* GazeboRosJointCommander::LoadJointSet(sdf::ElementPtr _sdf, int index) {
    using namespace std;
    ros_info(" loading joint set");
    sdf::ElementPtr set = _sdf->GetElement("jointSet" + to_string(index));
    std::string controlled_joint_name = set->Get<std::string>();
    std::vector<physics::JointPtr> joints;

    for(int i = 1; true; i++) {
      using namespace std;
      std::string joint_element = "jointSet" + to_string(index) + "Joint" + to_string(i);
      ros_info(" searching " + joint_element);
      if(!_sdf->HasElement(joint_element))
        break;
      std::string joint_name = _sdf->GetElement(joint_element)->Get<std::string>();
      ros_info(" found " + joint_element + " getting real joint ");
      physics::JointPtr joint = parent->GetJoint(joint_name);

      if(!joint) {
        ros_fatal("The joint \"" + joint_name + "\" does not exists");
        return nullptr;
      }

      ros_info(" adding " + joint_element );
      joints.push_back(joint);
    }

    return new JointSet(controlled_joint_name, joints);
  }


  // Load the controller
  void GazeboRosJointCommander::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    using namespace std;

    this->parent = _parent;
    this->world = _parent->GetWorld();

    ros_info("starting");

    ros_info("searching joint set");
    std::vector<JointSet*> joint_sets;

    for(int i = 1; true; i++) {
      if (!_sdf->HasElement("jointSet" + to_string(i))) {
        ros_info(" joint set finished");
        break;
      }
      else {
        JointSet* joint_set = LoadJointSet(_sdf, i);
        if(joint_set == nullptr) {
           gzthrow("Unable to load the joint set");
        } else {
          joint_sets.push_back(joint_set);
        }
      }
    }

    this->robot_namespace_ = "";
    ros_info("searching namespace");
    if (!_sdf->HasElement("robotNamespace")) {
      ros_info(PLUGIN_NAME + "Plugin missing <robotNamespace>, defaults to " + this->robot_namespace_);
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ros_info("Starting " + PLUGIN_NAME + " (ns = " + robot_namespace_ + " )" );

    ros_info("creating subscribers ");

    for (JointSet* joint_set : joint_sets) {
      ros_info("adding subscriber for " +  joint_set->GetName());
      auto controlled_joint = new ControlledJointSet(joint_set);
      ros::SubscribeOptions so =
      ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/" + joint_set->GetName(), 10,
          boost::bind(&ControlledJointSet::cmdPositionCallback, controlled_joint, _1),
          ros::VoidPtr(), &queue_);
      controlled_joint->rosSubscriber = rosnode_->subscribe(so);
      controlled_joints.push_back(controlled_joint);
    }

    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&GazeboRosJointCommander::QueueThread, this));
  }

  // Finalize the controller
  void GazeboRosJointCommander::Shutdown() {
    ros_info("shutting down");
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();

  }

void ControlledJointSet::cmdPositionCallback(const sensor_msgs::JointState::ConstPtr& cmd_msg) {
    ros_info("in cmd position callback of " + joint_set->GetName());
    const std::vector<physics::JointPtr> joints = joint_set->GetJoints();

    for(int i = 0; i < joints.size(); i++) {
      physics::JointPtr joint = joints[i];
      ros_info("setting position " + std::to_string(cmd_msg->position[i]));
      joint->SetPosition(0, cmd_msg->position[i]);
    }
  }

  void GazeboRosJointCommander::QueueThread() {
    static const double timeout = 0.01;

    ros_info("queue thread started. is rosnode ok: " + std::to_string(rosnode_->ok()));
    ros_info("queue thread started. is alive_ : " + std::to_string(alive_));
    while (alive_ && rosnode_->ok()) {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosJointCommander)
}
