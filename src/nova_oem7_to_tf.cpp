// publishef from novatel_oem7_msgs/msg/BESTUTM
// current pose and tf

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

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
        this->declare_parameter<float>("z_coord_offset_plus", -10.0);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("child_frame_id", "lexus3/base_link");
        // z_coord_ref_switch can be exact / zero_based / orig / orig_offset
        // exact: the Z coorindinate is always z_coord_exact_height param (must be set in this launch)
        // zero_based: Z coordinate starts from 0 and relative
        // orig: the original Z provided by the sensor
        // orig_offset: the original Z plus a fixed offset
        this->declare_parameter<std::string>("z_coord_ref_switch", "orig");
        this->declare_parameter<std::string>("pose_topic_pub", "lexus3/gps/nova/current_pose");
        this->declare_parameter<std::string>("utm_topic_sub", "lexus3/gps/nova/bestutm");
        this->declare_parameter<std::string>("inspva_topic_sub", "lexus3/gps/nova/inspva");
        this->declare_parameter<bool>("debug_publish", false);

        this->get_parameter("x_coord_offset", m_x_coord_offset_);
        this->get_parameter("y_coord_offset", m_y_coord_offset_);
        this->get_parameter("z_coord_exact_height", m_z_coord_exact_height_);
        this->get_parameter("z_coord_offset_plus", m_z_coord_orig_offset_);
        this->get_parameter("frame_id", m_frame_id_);
        this->get_parameter("child_frame_id", m_child_frame_id_);
        this->get_parameter("z_coord_ref_switch", m_z_coord_ref_switch_);
        this->get_parameter("pose_topic_pub", m_pose_topic_);
        this->get_parameter("utm_topic_sub", m_utm_topic_);
        this->get_parameter("inspva_topic_sub", m_inspva_topic_);
        this->get_parameter("debug_publish", debug_pub);

        m_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_pose_topic_, 10);
        // additional publisher for translated pose
        if (debug_pub)
        {
            m_pose_pub_debug_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose_debug_original", 10); // TODO: this is only temporary
        }
        m_utm_sub_ = this->create_subscription<novatel_oem7_msgs::msg::BESTUTM>(m_utm_topic_, 10, std::bind(&CurrentPoseFromTf::utm_callback, this, _1));
        m_inspva_sub_ = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(m_inspva_topic_, 10, std::bind(&CurrentPoseFromTf::inspva_callback, this, _1));
        m_tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        RCLCPP_INFO_STREAM(this->get_logger(), "nova_oem7_to_tf node started");
        RCLCPP_INFO_STREAM(this->get_logger(), "nova coord offset: " << m_x_coord_offset_ << ", " << m_y_coord_offset_ << ", " << m_z_coord_exact_height_);
        RCLCPP_INFO_STREAM(this->get_logger(), "nova frame_id: " << m_frame_id_ << ", child_frame_id: " << m_child_frame_id_);
        RCLCPP_INFO_STREAM(this->get_logger(), "nova utm_topic: " << m_utm_topic_);

        if (m_z_coord_ref_switch_.compare("exact") == 0)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "nova exact height (z): " << m_z_coord_exact_height_);
        }
        else if (m_z_coord_ref_switch_.compare("zero_based") == 0)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "nova zero based height (z)");
        }
        else if (m_z_coord_ref_switch_.compare("orig") == 0)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "nova original height (z)");
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "nova orig offset height (z): " << m_z_coord_orig_offset_);
        }
    }

private:
    // Callback function for the subscriber novatel_oem7_msgs
    void utm_callback(const novatel_oem7_msgs::msg::BESTUTM::SharedPtr msg)
    {
        if (first_run_z_coord)
        {
            m_z_coord_start_ = msg->height;
            first_run_z_coord = false;
        }
        // create current pose
        m_currentPose.header.stamp = msg->header.stamp;
        m_currentPose.header.frame_id = m_frame_id_;
        // # 'x_coord_offset': -697237.0, # map_gyor_0
        // # 'y_coord_offset': -5285644.0, # map_gyor_0
        // # 'x_coord_offset': -639770.0,
        // # 'y_coord_offset': -5195040.0, # map_zala_0
        m_currentPose.pose.position.x = msg->easting + m_x_coord_offset_;
        m_currentPose.pose.position.y = msg->northing + m_y_coord_offset_;
        if (m_z_coord_ref_switch_.compare("exact") == 0)
        {
            m_currentPose.pose.position.z = m_z_coord_exact_height_;
        }
        else if (m_z_coord_ref_switch_.compare("zero_based") == 0)
        {
            m_currentPose.pose.position.z = msg->height - m_z_coord_start_;
        }
        else if (m_z_coord_ref_switch_.compare("orig") == 0)
        {
            m_currentPose.pose.position.z = msg->height;
        }
        else
        {
            m_currentPose.pose.position.z = msg->height + m_z_coord_orig_offset_;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = m_frame_id_;
        transformStamped.child_frame_id = m_child_frame_id_;
        transformStamped.transform.translation.x = msg->easting + m_x_coord_offset_;
        transformStamped.transform.translation.y = msg->northing + m_y_coord_offset_;
        transformStamped.transform.translation.z = m_currentPose.pose.position.z;
        if (m_z_coord_ref_switch_.compare("exact") == 0)
        {
            transformStamped.transform.translation.z = m_z_coord_exact_height_;
        }
        else
        {
            transformStamped.transform.translation.z = msg->height; // original height
        }
        transformStamped.transform.rotation = m_currentPose.pose.orientation;
        // Publish tf
        m_tfBroadcaster_->sendTransform(transformStamped);
        // Publish the current pose
        // Publish a translated copy of the current pose on current_pose2
        geometry_msgs::msg::PoseStamped translated_pose = m_currentPose;
        // Apply translation in the pose local frame.
        const double yaw = tf2::getYaw(translated_pose.pose.orientation);

        constexpr double offset_x = -1.585;   // local forward/backward
        constexpr double offset_y = 0.2755;   // local left/right

        translated_pose.pose.position.x += offset_x * std::cos(yaw) - offset_y * std::sin(yaw);
        translated_pose.pose.position.y += offset_x * std::sin(yaw) + offset_y * std::cos(yaw);

        // // Apply translation: x = x - 1.5, y = y + 0.2 Faulty logic
        // translated_pose.pose.position.x = translated_pose.pose.position.x - 1.585;
        // translated_pose.pose.position.y = translated_pose.pose.position.y + 0.2755;
        if (debug_pub)
        {
            m_pose_pub_debug_->publish(m_currentPose);
        }
        m_pose_pub_->publish(translated_pose);
    }

    void inspva_callback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg)
    {

        // RCLCPP_INFO(this->get_logger(), "INS PVA: %f %f %f", msg->roll, msg->pitch, msg->azimuth);
        tf2::Quaternion oriQuater, oriRot, oriNew;
        double R = (msg->roll) * (M_PI / 180) * -1;
        double P = (msg->pitch) * (M_PI / 180) * -1;
        double Y = (msg->azimuth) * (M_PI / 180) - M_PI;
        oriQuater.setRPY(R, P, Y);
        oriRot.setRPY(0.0, 0.0, M_PI / 2.0);
        oriNew = oriRot * oriQuater;
        oriNew.normalize();
        m_currentPose.pose.orientation.w = oriNew.getW() * -1;
        m_currentPose.pose.orientation.x = oriNew.getY();
        m_currentPose.pose.orientation.y = oriNew.getX() * -1;
        m_currentPose.pose.orientation.z = oriNew.getZ();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub_debug_;
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
    double m_z_coord_start_ = 0.0;
    double m_z_coord_orig_offset_ = 0.0;
    std::string m_pose_topic_ = "/lexus3/gps/nova/current_pose";
    std::string m_utm_topic_ = "/lexus3/nova/bestutm";
    std::string m_inspva_topic_ = "/lexus3/nova/inspva";
    bool debug_pub = false;
    bool first_run_z_coord = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentPoseFromTf>());
    rclcpp::shutdown();
    return 0;
}