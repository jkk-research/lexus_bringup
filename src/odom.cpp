// publishes nav_msgs/Path and steering marker and km/h

#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Odom : public rclcpp::Node
{
public:
    Odom() : Node("odom_node")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        sub_steer_ = this->create_subscription<pacmod3_msgs::msg::SystemRptFloat>("lexus3/pacmod/steering_rpt", 10, std::bind(&Odom::vehicleSteeringCallback, this, _1));
        sub_speed_ = this->create_subscription<pacmod3_msgs::msg::VehicleSpeedRpt>("lexus3/pacmod/vehicle_speed_rpt", 10, std::bind(&Odom::vehicleSpeedCallback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Odom::loop, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Node started: " << this->get_name());
    }

private:
    // Callback for steering wheel messages
    void vehicleSteeringCallback(const pacmod3_msgs::msg::SystemRptFloat &steer_msg)
    {
        steering_angle = steer_msg.output / 14.8;
        steering_enabled = steer_msg.enabled;
    }

    // Callback for vehicle speed messages
    void vehicleSpeedCallback(const pacmod3_msgs::msg::VehicleSpeedRpt &speed_msg)
    {
        vehicle_speed_mps = speed_msg.vehicle_speed;
        current_time = (this->now()).seconds();
        vehicleOdomCalc();
    }

    void vehicleOdomCalc()
    {
        if (!first_run)
        {
            dt = current_time - prev_time;
            //RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << std::fixed << std::setprecision(3) << dt);
        }
        {
            // /lexus3/pacmod/parsed_tx/vehicle_speed_rpt is assumed 30 Hz
            dt = 0.033333;
        }
        if (first_run)
        {
            first_run = false;
        }
        odom_transform.header.stamp = this->get_clock()->now();
        odom_transform.header.frame_id = "map";
        odom_transform.child_frame_id = "lexus3/odom";
        odom_transform.transform.translation.x += dt * vehicle_speed_mps * cos(theta);
        odom_transform.transform.translation.y += dt * vehicle_speed_mps * sin(theta);
        theta += dt * vehicle_speed_mps / wheelbase * tan(steering_angle);
        tf2::Quaternion theta_quat;
        theta_quat.setRPY(0.0, 0.0, theta);
        odom_transform.transform.rotation.w = theta_quat.getW();
        odom_transform.transform.rotation.x = theta_quat.getX();
        odom_transform.transform.rotation.y = theta_quat.getY();
        odom_transform.transform.rotation.z = theta_quat.getZ();
        tf_broadcaster_->sendTransform(odom_transform);
        prev_time = current_time;
    }

    void loop()
    {
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr sub_steer_;
    rclcpp::Subscription<pacmod3_msgs::msg::VehicleSpeedRpt>::SharedPtr sub_speed_;
    std::string pose_topic, marker_topic, pose_frame;
    const double wheelbase = 2.789;
    double steering_angle, vehicle_speed_mps;
    long unsigned int path_size;
    bool steering_enabled;
    bool first_run = true, publish_steer_marker, publish_kmph;
    geometry_msgs::msg::TransformStamped odom_transform;
    std::string current_map = "empty";
    double theta = 0.0;
    double current_time, prev_time, dt;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odom>());
    rclcpp::shutdown();
    return 0;
}
