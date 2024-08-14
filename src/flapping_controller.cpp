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
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&FlappingController::OnUpdate,this));
            this->old_secs = ros::Time::now().toSec();

            if (_sdf->HasElement("namespace_model"))
            {
                this->namespace_model = _sdf->Get<std::string>("namespace_model");
            }
                
            // create topics
            std::string flapping_frequency = "/"+ this->namespace_model + "/frequency";
            std::string flapping_speed = "/"+ this->namespace_model + "/speed";

            // initiate ros
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "set_flapping speed_rosnode", ros::init_options::NoSigintHandler);
            
            }
            
            // creating the ros node
            this->rosNode.reset(new ros::NodeHandle("fly_rosnode"));

            //freq
            ros::SubscribeOptions so1 = 
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    flapping_frequency,
                    1,
                    boost::bind(&FlappingController::OnRosMsg_flapping_freq, this, _1),
                    ros::VoidPtr(), &this->rosQueue1);

            // speed publisher
            this->speed_pub = this->rosNode->advertise<std_msgs::Float32>("/bird2/speed", 10);

            // height subscriber
            this->height_sub = this->rosNode->subscribe("/bird2/height", 10, &FlappingController::shutdown, this);

            this->rosSub1 = this->rosNode->subscribe(so1);
            this->rosQueueThread1 = std::thread(std::bind(&FlappingController::QueueThread1, this));
            this->start_time = ros::Time::now().toSec();

            ROS_WARN("loaded flapping controller plugin with parent... %s", this->model->GetName().c_str());

        }

        public: void OnUpdate()
        {
            double new_secs = ros::Time::now().toSec();
            double delta = new_secs - this->old_secs;
            double max_delta = 0.0;
            if (this->freq_update != 0.0)
            {
                max_delta = 1.0 / this->freq_update;
            }
            //if (delta > max_delta && delta != 0.0)
            if (flap)
            {
                this->old_secs = new_secs;
                double current_time = ros::Time::now().toSec();
                double elapsed_time = current_time - this->start_time;

                double left_wing_velocity = this->flapping_amplitude * sin(2 * M_PI * this->flapping_freq * elapsed_time);
                double right_wing_velocity = this->flapping_amplitude * sin(2 * M_PI * this->flapping_freq * elapsed_time)*-1;

                this->model->GetJoint("joint_4")->SetPosition(0, left_wing_velocity);
                this->model->GetJoint("joint_6")->SetPosition(0, right_wing_velocity);

                //publishing to liftdragplugin
                this->speed.data = static_cast<double>(left_wing_velocity);
                speed_pub.publish(speed);
            }
        }

        public: void SetflappingFreq(const double &_mag)
        {
            this->flapping_freq = _mag;
            this->flapping_amplitude = 0.35;
            //ROS_INFO("flapping_freq >> %f", this->flapping_freq);
        }

        public: void shutdown(const std_msgs::Float32::ConstPtr &msg)
        {
            float height = msg->data;
            if (height < 0.1)
            {
                this->flap = false;
                this->model->GetJoint("joint_4")->SetPosition(0, 0);
                this->model->GetJoint("joint_6")->SetPosition(0, 0);
            }

            else
                this->flap = true;
        }

        public: void OnRosMsg_flapping_freq(const std_msgs::Float32ConstPtr &_msg)
        {
            this->SetflappingFreq(_msg->data);
        }

        // process msgs
        private: void QueueThread1()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue1.callAvailable(ros::WallDuration(timeout));
            }
        }

        private:
            physics::ModelPtr model;
            event::ConnectionPtr updateConnection;
            std::unique_ptr<ros::NodeHandle> rosNode;

            ros::Subscriber rosSub1;
            ros::CallbackQueue rosQueue1;
            std::thread rosQueueThread1;
            ros::Publisher speed_pub;
            ros::Subscriber height_sub;

            std::string namespace_model = "";
            std_msgs::Float32 speed;

            double flapping_freq;
            double flapping_amplitude;
            double start_time;
            double old_secs;
            double freq_update = 10.0;
            bool flap;
    };

    // register the plugin with gazebo
    GZ_REGISTER_MODEL_PLUGIN(FlappingController)
}
