#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector3.hh>

#include <iostream>
#include <string>

namespace gazebo
{
    class ThrusterPlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        std::string left_thruster_link;
        std::string right_thruster_link;
        physics::LinkPtr left_thruster;
        physics::LinkPtr right_thruster;
        std::shared_ptr<ros::NodeHandle> nh;
        ros::Subscriber sub;
        double left_force;
        double right_force;
        std::string ros_topic;
        event::ConnectionPtr updateConnection;

        void ApplyForces()
        {
            if (!this->left_thruster || !this->right_thruster)
            {
                std::cerr << "Thrusters not initialized. Cannot apply forces." << std::endl;
                return;
            }   

            this->left_thruster->AddForce(ignition::math::Vector3d(3000000000.0, 0.0, 0.0));
            this->right_thruster->AddForce(ignition::math::Vector3d(3000000000, 0.0, 0.0));
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
            if (_sdf->HasElement("left_thruster"))
            {
                this->left_thruster_link = _sdf->Get<std::string>("left_thruster");
                std::cerr << "Left Thruster Link: " << this->left_thruster_link << std::endl;
            }
            else
            {
                std::cerr << "No left_thruster element in SDF" << std::endl;
            }

            if (_sdf->HasElement("right_thruster"))
            {
                this->right_thruster_link = _sdf->Get<std::string>("right_thruster");
                std::cerr << "Right Thruster Link: " << this->right_thruster_link << std::endl;
            }
            else
            {
                std::cerr << "No right_thruster element in SDF" << std::endl;
            }

            this->left_thruster = this->model->GetLink(this->left_thruster_link);
            this->right_thruster = this->model->GetLink(this->right_thruster_link);

            // Add a check to retry getting the links if they are not found initially
            if (!this->left_thruster || !this->right_thruster)
            {
                ROS_WARN("Waiting for thruster links to be available...");
                ros::Duration(1.0).sleep();  // Sleep for a while and then retry
                this->left_thruster = this->model->GetLink(this->left_thruster_link);
                this->right_thruster = this->model->GetLink(this->right_thruster_link);
            }

            if (!this->left_thruster)
            {
                std::cerr << "Failed to find left_thruster link: " << this->left_thruster_link << std::endl;
            }
            if (!this->right_thruster)
            {
                std::cerr << "Failed to find right_thruster link: " << this->right_thruster_link << std::endl;
            }

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::onUpdate, this));
        }
    };
    GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)
}
