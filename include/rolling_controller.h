#ifndef ROLLING_CONTROLLER_H
#define ROLLING_CONTROLLER_H

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
    class RollingController : public ModelPlugin
    {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
            void OnUpdate();
            double computeRollMoment(const ignition::math::Vector3d &M, const ignition::math::Pose3d &pose);
            void applyRollControl(double control, const ignition::math::Pose3d &pose);
            void SetrollingPos(const double &_value);
            void momentCallback(const geometry_msgs::Vector3::ConstPtr &M);
            void OnRosMsg_roll_pos(const std_msgs::Float32ConstPtr &_msg);

        private:
            void QueueThread();

            physics::ModelPtr model;
            physics::WorldPtr world;
            physics::LinkPtr baseLink;

            event::ConnectionPtr updateConnection1;
            std::unique_ptr<ros::NodeHandle> rosNode3;
            ros::Subscriber rosSub;
            ros::Subscriber momentSub;
            ros::CallbackQueue rosQueue;
            std::thread rosQueueThread;
            std::string namespace_model = "";
            
            double old_secs;
            double freq_update = 10.0;
            double start_time;
            double rolling_pos = 0;
            double desired_angle;
            double lower_limit;
            double upper_limit;
            double current_angle;
            double desiredRollMoment = 0.0;

            common::PID rollPID;
            ignition::math::Vector3d upward = ignition::math::Vector3d(0, 0, 1);
            ignition::math::Vector3d forward = ignition::math::Vector3d(1, 0, 0);
            ignition::math::Vector3d moment;
            ignition::math::Pose3d pose;
    };
}

#endif 
