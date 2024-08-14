#ifndef FLAPPING_CONTROLLER_HH
#define FLAPPING_CONTROLLER_HH

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

namespace gazebo
{
    class FlappingController : public ModelPlugin
    {
        public:
            FlappingController() = default;
            virtual ~FlappingController() = default;

            virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            virtual void OnUpdate();
            void SetflappingFreq(const double &_mag);
            void shutdown(const std_msgs::Float32::ConstPtr &msg);
            void OnRosMsg_flapping_freq(const std_msgs::Float32ConstPtr &_msg);

        private:
            void QueueThread1();

        private:
            physics::ModelPtr model;
            event::ConnectionPtr updateConnection;
            std::unique_ptr<ros::NodeHandle> rosNode;

            ros::Subscriber rosSub1;
            ros::CallbackQueue rosQueue1;
            std::thread rosQueueThread1;
            ros::Publisher speed_pub;
            ros::Subscriber height_sub;

            std::string namespace_model;
            std_msgs::Float32 speed;

            double flapping_freq;
            double flapping_amplitude;
            double start_time;
            double old_secs;
            double freq_update;
            bool flap;
    };

}

#endif
