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
    class VelocityPlugin : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        std::string link_name;
        physics::LinkPtr base_link;

        std::shared_ptr<ros::NodeHandle> nh;
        ros::Subscriber sub;

        double linear_velocity = 0;
        double angular_velocity = 0;
        std::string ros_topic;

        // Callback function to handle incoming velocity commands
        void onRosMsg(const std_msgs::Float32MultiArray::ConstPtr& msg)
        {
            if (msg->data.size() >= 2)
            {
                this->linear_velocity = msg->data[0];
                this->angular_velocity = msg->data[1];
                ROS_INFO_STREAM("Received velocities - Linear: " << this->linear_velocity << ", Angular: " << this->angular_velocity);
            }
            else
            {
                ROS_ERROR("Received invalid velocity message.");
            }
        }

        // Update function called at every simulation iteration
        void onUpdate()
        {
            // Get the current pose of the link
            ignition::math::Pose3d pose = this->base_link->WorldPose();

            // Transform the linear velocity from body frame to world frame
            ignition::math::Vector3d linear_velocity_world = pose.Rot().RotateVector(ignition::math::Vector3d(0, -1 * this->linear_velocity, 0));

            // Apply linear velocity to the base link in world frame
            this->base_link->SetLinearVel(linear_velocity_world);

            // Angular velocity remains in the body frame
            this->base_link->SetAngularVel(ignition::math::Vector3d(0, 0, this->angular_velocity));
        }

    public:
        VelocityPlugin() : ModelPlugin(){}
        virtual ~VelocityPlugin(){}

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;

            if (_sdf->HasElement("link_name"))
            {
                this->link_name = _sdf->Get<std::string>("link_name");
                this->base_link = this->model->GetLink(this->link_name);

                if (!this->base_link)
                {
                    ROS_ERROR_STREAM("Link " << this->link_name << " not found in model.");
                    return;
                }
            }
            else
            {
                ROS_ERROR("No link name specified in SDF.");
                return;
            }

            if (_sdf->HasElement("ros_topic"))
            {
                this->ros_topic = _sdf->Get<std::string>("ros_topic");
            }
            else
            {
                ROS_ERROR("No ROS topic specified in SDF.");
                return;
            }

            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized. Unable to load plugin.");
                return;
            }

            this->nh.reset(new ros::NodeHandle(""));
            this->sub = this->nh->subscribe(this->ros_topic, 1, &VelocityPlugin::onRosMsg, this);

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&VelocityPlugin::onUpdate, this));
        }

    private:
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(VelocityPlugin)
}
