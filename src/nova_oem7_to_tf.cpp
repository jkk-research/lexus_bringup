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

        this->declare_parameter<float>("x_coord_offset", -697237.0);
        this->declare_parameter<float>("y_coord_offset", -5285644.0);
        this->declare_parameter<float>("z_coord_exact_height", 1.9);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("child_frame_id", "lexus3/base_link");
        this->declare_parameter<std::string>("z_coord_ref_switch", "orig");
        this->get_parameter("x_coord_offset", m_x_coord_offset_);
        this->get_parameter("y_coord_offset", m_y_coord_offset_);
        this->get_parameter("z_coord_exact_height", m_z_coord_exact_height_);
        this->get_parameter("frame_id", m_frame_id_);
        this->get_parameter("child_frame_id", m_child_frame_id_);
        this->get_parameter("z_coord_ref_switch", m_z_coord_ref_switch_);

       
        m_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lexus3/gps/nova/current_pose", 10);
        m_utm_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTUTM>("/novatel/oem7/bestutm", 10, std::bind(&CurrentPoseFromTf::utm_callback, this, _1));
        m_inspva_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>("/novatel/oem7/inspva", 10, std::bind(&CurrentPoseFromTf::inspva_callback, this, _1));
        m_tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO_STREAM(this->get_logger(), "nova_oem7_to_tf node started");
        RCLCPP_INFO_STREAM(this->get_logger(), "nova coord offset: " << m_x_coord_offset_ << ", " << m_y_coord_offset_ << ", " << m_z_coord_exact_height_);
        RCLCPP_INFO_STREAM(this->get_logger(), "nova frame_id: " << m_frame_id_ << ", child_frame_id: " << m_child_frame_id_);
        if (m_z_coord_ref_switch_.compare("exact") == 0){
            RCLCPP_INFO_STREAM(this->get_logger(), "nova exact height (z): " << m_z_coord_exact_height_);
        }
        else {
            RCLCPP_INFO_STREAM(this->get_logger(), "nova original height (z)");
        }

    }

private:

    // Callback function for the subscriber novatel_oem7_msgs
    void utm_callback(const novatel_oem7_msgs::msg::BESTUTM::SharedPtr msg)
    {       
        
        // create current pose
        m_currentPose.header.stamp = msg->header.stamp;
        m_currentPose.header.frame_id = m_frame_id_;
        // # 'x_coord_offset': -697237.0, # map_gyor_0
        // # 'y_coord_offset': -5285644.0, # map_gyor_0
        // # 'x_coord_offset': -639770.0, 
        // # 'y_coord_offset': -5195040.0, # map_zala_0
        m_currentPose.pose.position.x = msg->easting + m_x_coord_offset_; 
        m_currentPose.pose.position.y = msg->northing + m_y_coord_offset_; 
        if (m_z_coord_ref_switch_.compare("exact") == 0){
            m_currentPose.pose.position.z = m_z_coord_exact_height_; 
        }
        else {
            m_currentPose.pose.position.z = msg->height; // original height
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = m_frame_id_;
        transformStamped.child_frame_id = m_child_frame_id_;
        transformStamped.transform.translation.x = msg->easting + m_x_coord_offset_; 
        transformStamped.transform.translation.y = msg->northing + m_y_coord_offset_;
        transformStamped.transform.translation.z = m_currentPose.pose.position.z;
        if (m_z_coord_ref_switch_.compare("exact") == 0){
            transformStamped.transform.translation.z = m_z_coord_exact_height_; 
        }
        else {
            transformStamped.transform.translation.z = msg->height; // original height
        }
        transformStamped.transform.rotation = m_currentPose.pose.orientation;
        // Publish tf
        m_tfBroadcaster_->sendTransform(transformStamped);
        // Publish the current pose
        m_pose_pub_->publish(m_currentPose);
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
        m_currentPose.pose.orientation.w = oriNew.getW() * -1;
        m_currentPose.pose.orientation.x = oriNew.getY();
        m_currentPose.pose.orientation.y = oriNew.getX() * -1;
        m_currentPose.pose.orientation.z = oriNew.getZ();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTUTM>::SharedPtr m_utm_sub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr m_inspva_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster_;
    geometry_msgs::msg::PoseStamped m_currentPose;
    // TODO: parameters 
    std::string m_frame_id_ = "map";
    std::string m_child_frame_id_ = "lexus3/base_link";
    std::string m_z_coord_ref_switch_ = "orig"; // orig or exact
    double m_x_coord_offset_ = 0.0;
    double m_y_coord_offset_ = 0.0;
    double m_z_coord_exact_height_ = 1.9;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentPoseFromTf>());
    rclcpp::shutdown();
    return 0;
}