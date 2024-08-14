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
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->world = this->model->GetWorld();
            this->updateConnection1 = event::Events::ConnectWorldUpdateBegin(std::bind(&RollingController::OnUpdate,this));
            this->old_secs = ros::Time::now().toSec();

            if (_sdf->HasElement("namespace_model2"))
            {
                this->namespace_model = _sdf->Get<std::string>("namespace_model2");
            }
                
            // create topics 
            std::string rolling_position = "/"+ this->namespace_model + "/direction";

            // initiate ros
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "set_rolling_rosnode", ros::init_options::NoSigintHandler);
                ROS_INFO("ROS node initialized.");
            }
            
            // creating the ros node
            this->rosNode3.reset(new ros::NodeHandle("roll_rosnode"));
            
            // subscribing for moment msgs
            this->momentSub = this->rosNode3->subscribe("bird2/t4", 10, &RollingController::momentCallback, this);

            // pos
            ros::SubscribeOptions so3 = 
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    rolling_position,
                    5,
                    boost::bind(&RollingController::OnRosMsg_roll_pos, this, _1),
                    ros::VoidPtr(), &this->rosQueue);

            this->rosSub = this->rosNode3->subscribe(so3);
            this->rosQueueThread = std::thread(std::bind(&RollingController::QueueThread, this));
            this->start_time = ros::Time::now().toSec();
            this->lower_limit = this->model->GetJoint("joint_8")->LowerLimit(0);
            this->upper_limit = this->model->GetJoint("joint_8")->UpperLimit(0);
            this->current_angle = this->model->GetJoint("joint_8")->Position(0);
            this->baseLink = this->model->GetLink("dummy");

            // pid for flight stabilizing
            this->rollPID.Init(100, 10, 10, 0, 0, 10, -10);
            
            ROS_WARN("loaded rolling controller plugin with parent... %s", this->model->GetName().c_str());

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
                        if (this->rolling_pos == 1)
                        {
                            this->model->GetJoint("joint_8")->SetPosition(0, this->current_angle);
                            this->model->GetJoint("joint_10")->SetPosition(0, this->current_angle * -1);
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                        else if (this->rolling_pos == -1)
                        {
                            this->model->GetJoint("joint_10")->SetPosition(0, this->current_angle);
                            this->model->GetJoint("joint_8")->SetPosition(0, this->current_angle * -1);
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        }
                    }
                }
            }

            else
            {
                this->pose = this->model->WorldPose();

                // getting rolling moment
                double rollingMoment = this->computeRollMoment(this->moment, this->pose);

                // rolling error
                double rollError = this->desiredRollMoment - rollingMoment;

                // calculating roll effort
                double rollControl = this->rollPID.Update(rollError, this->world->Physics()->GetMaxStepSize());

                // apply roll effot
                this->applyRollControl(rollControl, this->pose);
            }
        }

        public: double computeRollMoment(const ignition::math::Vector3d &M, const ignition::math::Pose3d &pose)
        {
            ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
            ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);
            ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();
            return M.Dot(spanwiseI);
        }

        public: void applyRollControl(double control, const ignition::math::Pose3d &pose)
        {
            ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
            ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);
            ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();
            ignition::math::Vector3d rollTorque = control * spanwiseI;
            this->baseLink->AddTorque(rollTorque);
        }

        public: void SetrollingPos(const double &_value)
        {
            if (_value == -1)
                this->desired_angle = this->lower_limit;
            
            else if (_value == 1)
                this->desired_angle = this->upper_limit;
            
            else if (_value == 0)
                this->desired_angle = 0;

            else 
                ROS_WARN("wrong rolling direction");
        }

        public: void momentCallback(const geometry_msgs::Vector3::ConstPtr &M)
        {
            ignition::math::Vector3d m{M->x, M->y, M->z};
            this->moment = m;
        }

        public: void OnRosMsg_roll_pos(const std_msgs::Float32ConstPtr &_msg)
        {
            this->rolling_pos = _msg->data;
            this->SetrollingPos(rolling_pos);
        }

        // process msgs
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode3->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }
        private:
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

    // register the plugin with gazebo
    GZ_REGISTER_MODEL_PLUGIN(RollingController)
}
