// publishef from novatel_oem7_msgs/msg/BESTUTM
// current pose and tf

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"

#include "novatel_oem7_msgs/msg/bestutm.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CurrentPoseFromTf : public rclcpp::Node
{
public:
    CurrentPoseFromTf() : Node("nova_oem7_to_tf_node")
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lexus3/gps/nova/current_pose", 10);
        utm_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTUTM>("lexus3/gps/nova/bestutm", 10, std::bind(&CurrentPoseFromTf::utm_callback, this, _1));
        inspva_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>("lexus3/gps/nova/inspva", 10, std::bind(&CurrentPoseFromTf::inspva_callback, this, _1));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO_STREAM(this->get_logger(), "nova_oem7_to_tf node started");
    }

private:

    // Callback function for the subscriber novatel_oem7_msgs
    void utm_callback(const novatel_oem7_msgs::msg::BESTUTM::SharedPtr msg)
    {       
        
        // create current pose
        current_pose.header.stamp = msg->header.stamp;
        current_pose.header.frame_id = frame_id_;
        current_pose.pose.position.x = msg->easting;
        current_pose.pose.position.y = msg->northing;
        current_pose.pose.position.z = 1.0;

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = frame_id_;
        transformStamped.child_frame_id = child_frame_id_;
        transformStamped.transform.translation.x = msg->easting;
        transformStamped.transform.translation.y = msg->northing;
        transformStamped.transform.translation.z = current_pose.pose.position.z;
        transformStamped.transform.rotation = current_pose.pose.orientation;
        // Publish tf
        tf_broadcaster_->sendTransform(transformStamped);
        // Publish the current pose
        pose_pub_->publish(current_pose);
    }

    void inspva_callback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg)
    {

        // RCLCPP_INFO(this->get_logger(), "INS PVA: %f %f %f", msg->roll, msg->pitch, msg->azimuth);
        tf2::Quaternion oriQuater, oriRot, oriNew;
        double R = (msg->roll) * (M_PI / 180)  * -1;
        double P = (msg->pitch) * (M_PI / 180) * -1;
        double Y = (msg->azimuth) * (M_PI / 180) - M_PI;
        oriQuater.setRPY(R,P,Y); 
        oriRot.setRPY(0.0, 0.0, M_PI/2.0);
        oriNew = oriRot * oriQuater;
        oriNew.normalize();
        current_pose.pose.orientation.w = oriNew.getW() * -1;
        current_pose.pose.orientation.x = oriNew.getY();
        current_pose.pose.orientation.y = oriNew.getX() * -1;
        current_pose.pose.orientation.z = oriNew.getZ();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTUTM>::SharedPtr utm_sub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr inspva_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::PoseStamped current_pose;
    // TODO: parameters 
    std::string frame_id_ = "map";
    std::string child_frame_id_ = "lexus3/base_link";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentPoseFromTf>());
    rclcpp::shutdown();
    return 0;
}