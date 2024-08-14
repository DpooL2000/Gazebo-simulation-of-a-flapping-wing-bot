#ifndef LIFT_DRAG_PLUGIN_HH
#define LIFT_DRAG_PLUGIN_HH

#include <algorithm>
#include <functional>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <ros/callback_queue.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{
    class LiftDragPlugin : public ModelPlugin
    {
        public:
            LiftDragPlugin() = default;
            virtual ~LiftDragPlugin() = default;

            virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            virtual void OnUpdate();
            
        private:
            void QueueThread1();
            void frequencyCallback(const std_msgs::Float32::ConstPtr &msg);
            void speedCallback(const std_msgs::Float32::ConstPtr &msg);
            void positionCallback(const std_msgs::Float32::ConstPtr &msg);
            void directionCallback(const std_msgs::Float32::ConstPtr &msg);
            void statCallback(const std_msgs::Bool::ConstPtr &msg);

        private:
            physics::ModelPtr model;
            physics::LinkPtr baseLink;
            event::ConnectionPtr updateConnection;
            std::unique_ptr<ros::NodeHandle> rosNode;
            
            ros::Subscriber frequencySub1;
            ros::Subscriber speedSub1;
            ros::Subscriber positionSub1;
            ros::Subscriber directionSub1;
            ros::Subscriber statSub1;
            ros::Publisher l4;
            ros::Publisher heightPub;
            ros::Publisher velPub;
            ros::Publisher accelPub;
            ros::CallbackQueue rosQueue1;
            std::thread rosQueueThread1;

            double freq_update = 100;
            double cd = 0.144;
            double area = 0.154 * 2;
            double freq = 0.0;
            double pos = 0.0;
            double dir = 0.0;
            double speed = 0.0;
            double l = 0.18;
            double rho = 1.23;
            double A = 8.18 * 0.1;
            double V = 4.0; 
            double mp;
            double mr;
            double modelForce;
            double start_time;
            double old_secs;

            ignition::math::Vector3d velocity{4, 0, 0};
            ignition::math::Pose3d startpos{0, 0, 2, 0, 0, 0};
            ignition::math::Vector3d forward{1, 0 , 0};
            ignition::math::Vector3d upward{0, 0, 1};
            ignition::math::Vector3d zero{0, 0, 0};
            ignition::math::Vector3d totalTorque{0, 0, 0};

            std::string namespace_model;
            geometry_msgs::Vector3 totalTorqueMsg;
            std_msgs::Float32 heightMsg;
    };

}

#endif
