#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float32MultiArray.h> 

#include <iostream>
#include <string>

namespace gazebo
{
    class ThrusterPlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        std::string link_name;
        physics::LinkPtr base_link;

        std::shared_ptr<ros::NodeHandle> nh;
        ros::Subscriber sub;

        double left_force = 0;
        double right_force = 0;
        std::string ros_topic;
        ignition::math::Vector3d left_force_point;
        ignition::math::Vector3d right_force_point;
        
        event::ConnectionPtr updateConnection;

        void ApplyForces()
        {
            if (!this->base_link)
            {
                std::cerr << "Links not initialized. Cannot apply forces." << std::endl;
                return;
            }
            ignition::math::Vector3d left_force_world = this->base_link->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0, this->left_force, 0.0));
            ignition::math::Vector3d right_force_world = this->base_link->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0, this->right_force, 0.0));

            this->base_link->AddForceAtRelativePosition(left_force_world, this->left_force_point);
            this->base_link->AddForceAtRelativePosition(right_force_world, this->right_force_point);
        }
        
        void onROSMsg(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            if (msg->data.size() >= 2)
            {
                this->left_force = msg->data[0] * -1; //the frame is flipped, we need to invert.
                this->right_force = msg->data[1] * -1;
                ROS_INFO_STREAM("Received forces - Left: " << this->left_force << ", Right: " << this->right_force);
            }
            else
            {
                ROS_ERROR("Received invalid force message.");
            }
        }

        void onUpdate()
        {
            this->ApplyForces();
        }


    public:
        ThrusterPlugin() : ModelPlugin() {}
        virtual ~ThrusterPlugin() {}

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {

            this->model = _model;
            if (_sdf->HasElement("link_name"))
            {
                this->link_name = _sdf->Get<std::string>("link_name");
                std::cerr << "Base Link Name: " << this->link_name << std::endl;
            }
            else
            {
                std::cerr << "No link element in SDF" << std::endl;
            }
            this->base_link = this->model->GetLink(this->link_name);

            if (_sdf->HasElement("left_force_point"))
            {
                this->left_force_point = _sdf->GetElement("left_force_point")->GetElement("xyz")->Get<ignition::math::Vector3d>();
                ROS_INFO_STREAM("Left Force Point: " << this->left_force_point);
            }

            if (_sdf->HasElement("right_force_point"))
            {
                this->right_force_point = _sdf->GetElement("right_force_point")->GetElement("xyz")->Get<ignition::math::Vector3d>();
                ROS_INFO_STREAM("Right Force Point: " << this->right_force_point);
            }

            if (_sdf->HasElement("ros_topic"))
            {
                this->ros_topic = _sdf->Get<std::string>("ros_topic");
                ROS_INFO_STREAM("ROS Topic: " << this->ros_topic);
                this->nh.reset(new ros::NodeHandle(""));
                this->sub = this->nh->subscribe(this->ros_topic, 1, &ThrusterPlugin::onROSMsg, this);
            }
            else
            {
                ROS_ERROR("ThrusterPlugin: No ROS topic specified.");
                return;
            }



            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::onUpdate, this));
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)
}
