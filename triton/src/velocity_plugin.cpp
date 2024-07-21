#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

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
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;

        double linear_velocity = 0;
        double angular_velocity = 0;
        std::string ros_topic;
        std::string odom_topic;

        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        ros::Time last_time;

        // Callback function to handle incoming velocity commands
        void onRosMsg(const geometry_msgs::Twist::ConstPtr& msg)
        {
            this->linear_velocity = msg->linear.x;
            this->angular_velocity = msg->angular.z;
            ROS_INFO_STREAM("Received velocities - Linear: " << this->linear_velocity << ", Angular: " << this->angular_velocity);
        }

        // Update function called at every simulation iteration
        void onUpdate()
        {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - this->last_time).toSec();

            // Update the position based on the velocities
            double delta_x = this->linear_velocity * cos(this->theta) * dt;
            double delta_y = this->linear_velocity * sin(this->theta) * dt;
            double delta_theta = this->angular_velocity * dt;

            this->x += delta_x;
            this->y += delta_y;
            this->theta += delta_theta;

            // Set the linear and angular velocities on the base link
            ignition::math::Vector3d linear_velocity_world(this->linear_velocity * cos(this->theta), this->linear_velocity * sin(this->theta), 0);
            this->base_link->SetLinearVel(linear_velocity_world);
            this->base_link->SetAngularVel(ignition::math::Vector3d(0, 0, this->angular_velocity));

            // Publish the odometry
            this->publishOdometry(current_time, linear_velocity_world, ignition::math::Vector3d(0, 0, this->angular_velocity));

            this->last_time = current_time;
        }

        void publishOdometry(ros::Time current_time, ignition::math::Vector3d linear_velocity, ignition::math::Vector3d angular_velocity)
        {
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = this->link_name;

            // Set the position
            odom.pose.pose.position.x = this->x;
            odom.pose.pose.position.y = this->y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->theta);

            // Set the velocity
            odom.twist.twist.linear.x = linear_velocity.X();
            odom.twist.twist.linear.y = linear_velocity.Y();
            odom.twist.twist.angular.z = angular_velocity.Z();

            // Publish the message
            this->odom_pub.publish(odom);

            // Publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = this->link_name;

            odom_trans.transform.translation.x = this->x;
            odom_trans.transform.translation.y = this->y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(this->theta);

            this->odom_broadcaster.sendTransform(odom_trans);
        }

    public:
        VelocityPlugin() : ModelPlugin(), x(0.0), y(0.0), theta(0.0) {}
        virtual ~VelocityPlugin() {}

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

            if (_sdf->HasElement("odom_topic"))
            {
                this->odom_topic = _sdf->Get<std::string>("odom_topic");
            }
            else
            {
                ROS_ERROR("No odometry topic specified in SDF.");
                return;
            }

            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized. Unable to load plugin.");
                return;
            }

            this->nh.reset(new ros::NodeHandle(""));
            this->sub = this->nh->subscribe(this->ros_topic, 1, &VelocityPlugin::onRosMsg, this);
            std::cout << this->ros_topic << std::endl;
            this->odom_pub = this->nh->advertise<nav_msgs::Odometry>(this->odom_topic, 1);

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&VelocityPlugin::onUpdate, this));

            this->last_time = ros::Time::now();
        }

    private:
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_MODEL_PLUGIN(VelocityPlugin)
}
