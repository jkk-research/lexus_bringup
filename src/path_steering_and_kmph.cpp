// publishes nav_msgs/Path and steering marker and km/h

#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>

#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace locations // add when new locations appear
{
    enum LOCATIONS
    {
        DEFAULT = 0,
        GYOR,
        ZALA,
        NOT_USED_YET
    };
}
class PathSteerAndKmph : public rclcpp::Node
{
public:
    PathSteerAndKmph() : Node("path_steering_kmph_node")
    {
        this->declare_parameter<std::string>("pose_topic", "lexus3/current_pose");
        this->declare_parameter<std::string>("pose_frame", "lexus3/base_link");
        this->declare_parameter<std::string>("marker_topic", "marker_steering");
        this->declare_parameter<std::string>("path_topic", "marker_path");
        this->declare_parameter<std::string>("marker_color", "y");
        this->declare_parameter<bool>("publish_steer_marker", true);
        this->declare_parameter<bool>("publish_kmph", true);
        this->declare_parameter<int>("path_size", 1500);

        this->get_parameter("pose_topic", pose_topic);
        this->get_parameter("pose_frame", pose_frame);
        this->get_parameter("marker_topic", marker_topic);
        this->get_parameter("path_topic", path_topic);
        this->get_parameter("marker_color", marker_color);
        this->get_parameter("publish_steer_marker", publish_steer_marker);
        this->get_parameter("path_size", path_size);
        this->get_parameter("publish_kmph", publish_kmph);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        if (publish_kmph)
        {
            sub_steer_ = this->create_subscription<pacmod3_msgs::msg::SystemRptFloat>("lexus3/pacmod/steering_rpt", 10, std::bind(&PathSteerAndKmph::vehicleSteeringCallback, this, _1));
            sub_speed_ = this->create_subscription<pacmod3_msgs::msg::VehicleSpeedRpt>("lexus3/pacmod/vehicle_speed_rpt", 10, std::bind(&PathSteerAndKmph::vehicleSpeedCallback, this, _1));
        }
        // sub_current_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&PathSteerAndKmph::vehiclePoseCallback, this, _1));
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 1);
        path_pub = this->create_publisher<nav_msgs::msg::Path>(path_topic, 1);
        if (publish_kmph)
        {
            speed_kmph_pub = this->create_publisher<std_msgs::msg::Float32>("lexus3/vehicle_speed_kmph", 1);
        }
        // Call loop function 20 Hz (50 milliseconds)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PathSteerAndKmph::loop, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Node started: " << this->get_name() << " subscribed: " << pose_topic << " publishing: " << marker_topic << " " << path_topic);
    }

private:
    int getLocation(const geometry_msgs::msg::PoseStamped &pose)
    {
        double distance_from_gyor = std::sqrt(std::pow(pose.pose.position.x - map_gyor_0_x, 2) + std::pow(pose.pose.position.y - map_gyor_0_y, 2));
        double distance_from_zala = std::sqrt(std::pow(pose.pose.position.x - map_zala_0_x, 2) + std::pow(pose.pose.position.y - map_zala_0_y, 2));
        // RCLCPP_INFO_STREAM(this->get_logger(), "distance_from_gyor: " << distance_from_gyor << " distance_from_zala: " << distance_from_zala);
        if (distance_from_gyor < 50000.0)
        { // 50 km from Gyor
            RCLCPP_INFO_STREAM(this->get_logger(), "Welcome to Gyor");
            return locations::GYOR;
        }
        else if (distance_from_zala < 50000.0)
        { // 50 km from Zalaegerszeg
            RCLCPP_INFO_STREAM(this->get_logger(), "Welcome to Zala");
            return locations::ZALA;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Default map");
            return locations::DEFAULT;
        }
    }

    // Callback for steering wheel messages
    void vehicleSteeringCallback(const pacmod3_msgs::msg::SystemRptFloat &steer_msg)
    {
        // between  0.0 and 10.1 m/s 0.182 ratio
        // between 10.1 and 10.1 m/s 0.182-0.120 slope ratio
        // between 19.3 and max  m/s 0.120 ratio
        // TODO:
        // if (vehicle_speed_mps < 10.1)
        //{
        steering_angle = steer_msg.output / 14.8;
        //}
        /*
        else if (vehicle_speed_mps < 19.3)
        {
            // slope which is 0.182 at 10.1 and 0.120 at 19.3
            double ratio = (0.120 - 0.182) / (19.3 - 10.1) * (steer_msg.output - 10.1) + 0.182;
            steering_angle = steer_msg.output * ratio;
        }
        else
        {
            steering_angle = steer_msg.output * 0.120;
        }
        */
        steering_enabled = steer_msg.enabled;
    }

    // Callback for pose messages
    void vehiclePoseCallback(const geometry_msgs::msg::PoseStamped &pos_msg)
    {
        actual_pose = pos_msg;
        // RCLCPP_INFO_STREAM(this->get_logger(), "New pos ");
    }
    // Callback for vehicle speed messages
    void vehicleSpeedCallback(const pacmod3_msgs::msg::VehicleSpeedRpt &speed_msg)
    {
        vehicle_speed_mps = speed_msg.vehicle_speed;
        std_msgs::msg::Float32 speed_kmph;
        speed_kmph.data = vehicle_speed_mps * 3.6;
        speed_kmph_pub->publish(speed_kmph);
    }

    // get tf2 transform from map to lexus3/base_link
    void vehiclePoseFromTransform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform("map", pose_frame, tf2::TimePointZero);
        }

        catch (const tf2::TransformException &ex)
        {
            // RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }
        actual_pose.pose.position.x = transformStamped.transform.translation.x;
        actual_pose.pose.position.y = transformStamped.transform.translation.y;
        actual_pose.pose.position.z = transformStamped.transform.translation.z;
        actual_pose.pose.orientation.x = transformStamped.transform.rotation.x;
        actual_pose.pose.orientation.y = transformStamped.transform.rotation.y;
        actual_pose.pose.orientation.z = transformStamped.transform.rotation.z;
        actual_pose.pose.orientation.w = transformStamped.transform.rotation.w;
        actual_pose.header.stamp = this->now();
        actual_pose.header.frame_id = "map";
        // RCLCPP_INFO_STREAM(this->get_logger(), "actual_pose: " << actual_pose.pose.position.x << ", " << actual_pose.pose.position.y << ", " << actual_pose.pose.position.z);
    }

    void loop()
    {
        vehiclePoseFromTransform();
        if (publish_steer_marker)
        {
            visualization_msgs::msg::Marker steer_marker;
            steer_marker.header.frame_id = "lexus3/base_link";
            steer_marker.header.stamp = this->now();
            steer_marker.ns = "steering_path";
            steer_marker.id = 0;
            steer_marker.type = steer_marker.LINE_STRIP;
            steer_marker.action = visualization_msgs::msg::Marker::MODIFY;
            steer_marker.pose.position.x = 0;
            steer_marker.pose.position.y = 0;
            steer_marker.pose.position.z = 0;
            steer_marker.pose.orientation.x = 0.0;
            steer_marker.pose.orientation.y = 0.0;
            steer_marker.pose.orientation.z = 0.0;
            steer_marker.pose.orientation.w = 1.0;
            steer_marker.scale.x = 0.6;
            // https://github.com/jkk-research/colors
            if (marker_color == "r") // red
            {
                steer_marker.color.r = 0.96f;
                steer_marker.color.g = 0.22f;
                steer_marker.color.b = 0.06f;
            }
            else if (marker_color == "g") // green
            {
                steer_marker.color.r = 0.30f;
                steer_marker.color.g = 0.69f;
                steer_marker.color.b = 0.31f;
            }
            else if (marker_color == "b") // blue
            {
                steer_marker.color.r = 0.02f;
                steer_marker.color.g = 0.50f;
                steer_marker.color.b = 0.70f;
            }
            else if (marker_color == "k") // black
            {
                steer_marker.color.r = 0.19f;
                steer_marker.color.g = 0.19f;
                steer_marker.color.b = 0.23f;
            }
            else if (marker_color == "w") // white
            {
                steer_marker.color.r = 0.89f;
                steer_marker.color.g = 0.89f;
                steer_marker.color.b = 0.93f;
            }
            else if (marker_color == "p") // pink
            {
                steer_marker.color.r = 0.91f;
                steer_marker.color.g = 0.12f;
                steer_marker.color.b = 0.39f;
            }
            else
            { // yellow
                steer_marker.color.r = 0.94f;
                steer_marker.color.g = 0.83f;
                steer_marker.color.b = 0.07f;
            }
            steer_marker.color.a = 1.0;
            steer_marker.lifetime = rclcpp::Duration::from_seconds(0);
            double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
            for (int i = 0; i < 200; i++)
            {
                marker_pos_x += 0.01 * 10 * cos(theta);
                marker_pos_y += 0.01 * 10 * sin(theta);
                theta += 0.01 * 10 / wheelbase * tan(steering_angle);
                geometry_msgs::msg::Point p;
                p.x = marker_pos_x;
                p.y = marker_pos_y;
                steer_marker.points.push_back(p);
            }
            marker_pub->publish(steer_marker);
            steer_marker.points.clear();
        }
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        if ((actual_pose.pose.position.x > 0.001 || actual_pose.pose.position.x < -0.001) && !std::isnan(actual_pose.pose.position.y) && !std::isinf(actual_pose.pose.position.y))
        {
            if (first_run)
            {
                location = getLocation(actual_pose);
                first_run = false;
            }
            switch (location)
            {
            case locations::DEFAULT:
                pose.pose.position = actual_pose.pose.position;
                current_map = "map";
                break;
            case locations::GYOR:
                pose.pose.position.x = actual_pose.pose.position.x - map_gyor_0_x;
                pose.pose.position.y = actual_pose.pose.position.y - map_gyor_0_y;
                current_map = "map_gyor_0";
                break;
            case locations::ZALA:
                pose.pose.position.x = actual_pose.pose.position.x - map_zala_0_x;
                pose.pose.position.y = actual_pose.pose.position.y - map_zala_0_y;
                current_map = "map_zala_0";
                break;
            }
            pose.pose.position.z = actual_pose.pose.position.z;
            pose.header.frame_id = current_map;
            path.header.frame_id = current_map;
            pose.pose.orientation = actual_pose.pose.orientation;
            path.poses.push_back(pose);
            path.header.stamp = this->now();
        }
        path.poses.push_back(pose);
        // keep only the last n (path_size) path message
        if (path.poses.size() > path_size)
        {
            int shift = path.poses.size() - path_size;
            path.poses.erase(path.poses.begin(), path.poses.begin() + shift);
        }

        path_pub->publish(path);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr sub_steer_;
    rclcpp::Subscription<pacmod3_msgs::msg::VehicleSpeedRpt>::SharedPtr sub_speed_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_current_pose_;
    std::string pose_topic, marker_topic, path_topic, pose_frame;
    const double wheelbase = 2.789;
    double steering_angle, vehicle_speed_mps;
    long unsigned int path_size;
    bool steering_enabled;
    bool first_run = true, publish_steer_marker, publish_kmph;
    const double map_gyor_0_x = 697237.0, map_gyor_0_y = 5285644.0;
    const double map_zala_0_x = 639770.0, map_zala_0_y = 5195040.0;
    std::string marker_color;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_kmph_pub;
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped actual_pose;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::string current_map = "empty";
    int location = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSteerAndKmph>());
    rclcpp::shutdown();
    return 0;
}
