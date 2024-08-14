#ifndef PITCHING_CONTROLLER_H
#define PITCHING_CONTROLLER_H

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
    class PitchingController : public ModelPlugin
    {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
            void OnUpdate();
            double computePitchMoment(const ignition::math::Vector3d &M, const ignition::math::Pose3d &pose);
            void applyPitchControl(double controlInput, const ignition::math::Pose3d &pose);
            void SetpitchingPos(const double &_value);
            void momentCallback(const geometry_msgs::Vector3::ConstPtr &M);
            void OnRosMsg_pitch_pos(const std_msgs::Float32ConstPtr &_msg);

        private:
            void QueueThread();

            physics::ModelPtr model;
            physics::LinkPtr baseLink;
            physics::WorldPtr world;

            event::ConnectionPtr updateConnection1;
            std::unique_ptr<ros::NodeHandle> rosNode2;
            ros::Subscriber rosSub;
            ros::Subscriber momentSub;
            ros::CallbackQueue rosQueue;
            std::thread rosQueueThread;
            std::string namespace_model = "";
            
            double old_secs;
            double freq_update = 10.0;
            double start_time;
            double pitching_pos = 0;
            double desired_angle;
            double lower_limit;
            double upper_limit;
            double current_angle;
            double desiredPitchMoment = 0.0;

            common::PID pitchPID;
            ignition::math::Vector3d upward = ignition::math::Vector3d(0, 0, 1);
            ignition::math::Vector3d forward = ignition::math::Vector3d(1, 0, 0);
            ignition::math::Vector3d moment;
            ignition::math::Pose3d pose;
    };

}

#endif 
