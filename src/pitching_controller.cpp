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
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->world = this->model->GetWorld();
            this->updateConnection1 = event::Events::ConnectWorldUpdateBegin(std::bind(&PitchingController::OnUpdate,this));
            this->old_secs = ros::Time::now().toSec();

            if (_sdf->HasElement("namespace_model1"))
            {
                this->namespace_model = _sdf->Get<std::string>("namespace_model1");
            }
                
            // create topics 
            std::string pitching_position = "/"+ this->namespace_model + "/position";

            // initiate ros
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "set_pitching_position_rosnode", ros::init_options::NoSigintHandler);
                ROS_INFO("ROS node initialized.");
            }
            
            // creating the ros node
            this->rosNode2.reset(new ros::NodeHandle("pitch_rosnode"));

            //freq
            ros::SubscribeOptions so = 
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    pitching_position,
                    5,
                    boost::bind(&PitchingController::OnRosMsg_pitch_pos, this, _1),
                    ros::VoidPtr(), &this->rosQueue);

            this->rosSub = this->rosNode2->subscribe(so);
            this->rosQueueThread = std::thread(std::bind(&PitchingController::QueueThread, this));
            this->start_time = ros::Time::now().toSec();
            this->lower_limit = this->model->GetJoint("joint_8")->LowerLimit(0);
            this->upper_limit = this->model->GetJoint("joint_8")->UpperLimit(0);
            this->current_angle = this->model->GetJoint("joint_8")->Position(0);
            this->baseLink = this->model->GetLink("dummy");

            // pid for flight stabilizing
            this->pitchPID.Init(100, 10, 10, 0, 0, 10, -10);
            
            ROS_WARN("loaded pitching controller plugin with parent... %s", this->model->GetName().c_str());

        }

        public: void OnUpdate()
        {
            double new_secs = ros::Time::now().toSec();
            double delta = new_secs - this->old_secs;
            double max_delta = 0;
            double step = 0;

            if (this->current_angle != this->desired_angle)
            {
                if (this->desired_angle > this->current_angle)
                    step = (this->desired_angle - this->current_angle) / 10.0;
                
                else if (this->desired_angle < this->current_angle)
                    step = (this->current_angle - this->desired_angle) / -10.0;

                if (this->freq_update != 0.0)
                    max_delta = 1.0 / this->freq_update;
                
                //if (delta > max_delta && delta != 0.0)
                if (1)
                {
                    this->old_secs = new_secs;

                    for (int i = 0; i < 10; i++)
                    {
                        this->current_angle += step;

                        if ((step > 0 && this->current_angle > this->desired_angle) || (step < 0 && this->current_angle < this->desired_angle))
                        {
                            this->current_angle = this->desired_angle;
                            break;
                        }

                        this->model->GetJoint("joint_8")->SetPosition(0, this->current_angle);
                        this->model->GetJoint("joint_10")->SetPosition(0, this->current_angle);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                }
            }

            else
            {
                this->pose = this->baseLink->WorldPose();

                // getting pitching moment
                double rollingMoment = this->computePitchMoment(this->moment, this->pose);

                // pitching error
                double pitchError = this->desiredPitchMoment - rollingMoment;

                // calculating pitch effort
                double rollControl = this->pitchPID.Update(pitchError, this->world->Physics()->GetMaxStepSize());

                // apply pitch effot
                this->applyPitchControl(rollControl, this->pose);
            }
        }

        public: double computePitchMoment(const ignition::math::Vector3d &M, const ignition::math::Pose3d &pose)
        {
            ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
            return M.Dot(forwardI);
        }

        public: void applyPitchControl(double controlInput, const ignition::math::Pose3d &pose)
        {
            ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);
            ignition::math::Vector3d pitchTorque = controlInput * upwardI;
            this->baseLink->AddTorque(pitchTorque);
        }

        public: void SetpitchingPos(const double &_value)
        {
            if (_value == 0)
                this->desired_angle = 0;
            
            else if (_value == -1)
                this->desired_angle = this->lower_limit;
            
            else if (_value == 1)
                this->desired_angle = this->upper_limit;
            
            else 
                ROS_WARN("wrong pitching positon");
            
            //ROS_INFO("pitching position: %f", this->pitching_pos);
        }

        public: void momentCallback(const geometry_msgs::Vector3::ConstPtr &M)
        {
            ignition::math::Vector3d m{M->x, M->y, M->z};
            this->moment = m;
        }

        public: void OnRosMsg_pitch_pos(const std_msgs::Float32ConstPtr &_msg)
        {
            this->pitching_pos = _msg->data;
            this->SetpitchingPos(pitching_pos);
        }

        // process msgs
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode2->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private:
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

    // register the plugin with gazebo
    GZ_REGISTER_MODEL_PLUGIN(PitchingController)
}
