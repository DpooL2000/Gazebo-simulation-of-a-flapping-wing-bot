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
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&LiftDragPlugin::OnUpdate, this));
            this->old_secs = ros::Time::now().toSec();

            if (_sdf->HasElement("namespace_model3"))
            {
                this->namespace_model = _sdf->Get<std::string>("namespace_model3");
            }

            // creating rostopic
            std::string running = "/" + this->namespace_model + "/states";

            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "set_force_moment_rosnode", ros::init_options::NoSigintHandler);
            }

            // create ROS node
            this->rosNode.reset(new ros::NodeHandle(this->namespace_model));

            // subscribe to ROS topics
            this->frequencySub1 = rosNode->subscribe("/bird2/frequency", 10, &LiftDragPlugin::frequencyCallback, this);
            this->positionSub1 = rosNode->subscribe("/bird2/position", 10, &LiftDragPlugin::positionCallback, this);
            this->directionSub1 = rosNode->subscribe("/bird2/direction", 10, &LiftDragPlugin::directionCallback, this);
            this->statSub1 = rosNode->subscribe("/bird2/states", 10, &LiftDragPlugin::statCallback, this);
            this->start_time = ros::Time::now().toSec();

            // publishers for ros topics
            this->l4 = rosNode->advertise<geometry_msgs::Vector3>("/bird2/t4", 1);
            this->heightPub = rosNode->advertise<std_msgs::Float32>("/bird2/height", 1);
            this->velPub = rosNode->advertise<geometry_msgs::Vector3>("/bird2/velocity", 1);
            this->accelPub = rosNode->advertise<geometry_msgs::Vector3>("/bird2/acceleration", 1);

            // trying multi threading to increase the perfomance
            ros::SubscribeOptions so1 = 
                ros::SubscribeOptions::create<std_msgs::Float32>(
                "/bird2/speed",
                1,
                boost::bind(&LiftDragPlugin::speedCallback, this, _1),
                ros::VoidPtr(), &this->rosQueue1);

            this->speedSub1 = this->rosNode->subscribe(so1);

            // start queue thread
            this->rosQueueThread1 = std::thread(std::bind(&LiftDragPlugin::QueueThread1, this));

            // getting base link
            this->baseLink = model->GetLink("dummy");

            // get all links of the model
            double totalMass = 0.0;
            for (auto link : model->GetLinks())
            {
                double linkMass = link->GetInertial()->Mass();
                totalMass += linkMass;
            }

            double gravity = 9.81;
            double totalWeight = totalMass * gravity;

            ROS_WARN("loaded lift drag controller plugin with parent... %s", this->model->GetName().c_str());
            ROS_INFO("Total weight of the model: %f N", totalWeight);
        }

        public: void OnUpdate()
        {
            double new_secs = ros::Time::now().toSec();
            double delta = new_secs - this->old_secs;
            double max_delta = 0.0;
            double current_time = ros::Time::now().toSec();
            double elapsed_time = current_time - this->start_time;
            this->old_secs = new_secs;
            this->totalTorque = 0;

            // resulting moment
            ignition::math::Vector3d torque1(mr, mp ,0);
            ignition::math::Pose3d pose = this->baseLink->WorldPose();
            ignition::math::Vector3d worldTorque = pose.Rot().RotateVector(torque1);
            this->baseLink->AddTorque(worldTorque);

            // drag force calculations
            ignition::math::Vector3d vel = this->baseLink->WorldLinearVel();
            ignition::math::Vector3d velI = vel;
            velI.Normalize();

            // direction vectors
            ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
            ignition::math::Vector3d upwardI =pose.Rot().RotateVector(this->upward);
            ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

            // removing spanwise velocity from vel
            ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*velI;

            // get direction of drag
            ignition::math::Vector3d dragI = -velInLDPlane;
            dragI.Normalize();

            // get direction of lift
            ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
            liftI.Normalize();

            // compute dynamic pressure
            double speedInLDPlane = velInLDPlane.Length();
            double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

            // drag at cp
            ignition::math::Vector3d D = this->cd * q * this->area * dragI;

            // using pi * F * sin(2*pi*t) to model time average force
            double lift = this->modelForce + sqrt(2) * sin(2 * M_PI * elapsed_time);
            ignition::math::Vector3d L = lift * liftI;

            // apply forces at cp
            ignition::math::Vector3d F = L + D;
            F.Correct();
            this->baseLink->AddForce(F);

            for (auto link : this->model->GetLinks())
            {
                ignition::math::Vector3d torque2 = -link->WorldTorque();
                this->totalTorque += torque2;

                if (link->GetName() == "dummy")
                {
                    double height = pose.Pos().Z();
                    std_msgs::Float32 heightMsg;
                    heightMsg.data = height;
                    this->heightPub.publish(heightMsg);

                    ignition::math::Vector3d velocity = baseLink->WorldLinearVel();
                    ignition::math::Vector3d acceleration = baseLink->WorldLinearAccel();

                    geometry_msgs::Vector3 velMsg;
                    velMsg.x = velocity.X();
                    velMsg.y = velocity.Y();
                    velMsg.z = velocity.Z();
                    this->velPub.publish(velMsg);

                    geometry_msgs::Vector3 accelMsg;
                    accelMsg.x = acceleration.X();
                    accelMsg.y = acceleration.Y();
                    accelMsg.z = acceleration.Z();
                    this->accelPub.publish(accelMsg);
                }
            }
            
            totalTorqueMsg.x = totalTorque.X();
            totalTorqueMsg.y = totalTorque.Y();
            totalTorqueMsg.z = totalTorque.Z();
            
            this->l4.publish(totalTorqueMsg);
        }

        private: void QueueThread1()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue1.callAvailable(ros::WallDuration(timeout));
            }
        }

        private: void frequencyCallback(const std_msgs::Float32::ConstPtr &msg)
        {
            this->freq = msg->data;

            // these are the current model forces for lift in Newtons
            // [0.0008872] [0.06861017] [1.70014306] [3.8030878] [4.29144139] [6.6919642] [8.24759058] [10.46655156]

            if (this->freq == 1)
                this->modelForce = 0.0008872;

            else if (this->freq == 2)
                this->modelForce = 0.06861017;

            else if (this->freq == 3)
                this->modelForce = 1.70014306;

            else if (this->freq == 4)
                this->modelForce = 3.8030878;

            else if (this->freq == 5)
                this->modelForce = 4.29144139;

            else if (this->freq == 6)
                this->modelForce = 6.6919642;

            else if (this->freq == 7)
                this->modelForce = 8.24759058;

            else if (this->freq == 8)
                this->modelForce = 10.46655156;

        }

        private: void speedCallback(const std_msgs::Float32::ConstPtr &msg)
        {
            this->speed = msg->data;
        }

        private: void positionCallback(const std_msgs::Float32::ConstPtr &msg)
        {
            this->pos = msg->data;
            ignition::math::Vector3d velocity = this->model->WorldLinearVel();
            
            // apply pitching moments m1, m2
            if (this->pos == 1)
            {
                this->mp = 0.0237 * l * rho * V * V * A;
                ignition::math::Quaterniond pitchRotation(ignition::math::Vector3d::UnitY, 0.7);
                velocity = pitchRotation.RotateVector(velocity);
                this->model->SetLinearVel(velocity);
                    
            }
                
            else if (this->pos == -1)
            {
                this->mp = -0.0237 * l * rho * V * V * A;
                ignition::math::Quaterniond pitchRotation(ignition::math::Vector3d::UnitY, -0.7);
                velocity = pitchRotation.RotateVector(velocity);
                this->model->SetLinearVel(velocity);
            }
                
            else if (this->pos == 0)
            {
                this->mp = 0;
                this->model->SetLinearVel(this->velocity);
            }
        }

        private: void directionCallback(const std_msgs::Float32::ConstPtr &msg)
        {
            this->dir = msg->data;

            // apply rolling moments m3, m4
                if (this->dir == 1)
                    this->mr = 0.0118 * l * rho * V * V * A;
                
                else if (this->dir == -1)
                    this->mr = -0.0188 * l * rho * V * V * A;
                
                else if (this->dir == 0)
                    this->mr = 0;
        }

        private: void statCallback(const std_msgs::Bool::ConstPtr &msg)
        {
            // get model to jump in to start position
            if (msg->data == true)
            {
                this->model->SetWorldPose(this->startpos);
                this->model->SetLinearVel(this->velocity);
            }
            else if (msg->data == false)
            {}
        }
        
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
            ros::Publisher l6;
            ros::Publisher l8;
            ros::Publisher l10;
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

    GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)
}
