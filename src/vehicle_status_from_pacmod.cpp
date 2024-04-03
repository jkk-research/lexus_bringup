// This node creates a /vehicle_status (geometry_msgs/TwistStamped) from pacmod3_msgs::msg::VehicleSpeedRpt and pacmod3_msgs::msg::SystemRptFloat messages.


#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
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


class VehicleStatusPacmod : public rclcpp::Node
{
public:
    VehicleStatusPacmod() : Node("vehicle_status_from_pacmod_node")
    {
        this->declare_parameter<std::string>("steering_topic", "lexus3/pacmod/steering_rpt");
        this->declare_parameter<std::string>("speed_topic", "lexus3/pacmod/vehicle_speed_rpt");
        this->declare_parameter<std::string>("status_topic", "lexus3/vehicle_status");

        this->get_parameter("steering_topic", steering_topic);
        this->get_parameter("speed_topic", speed_topic);
        this->get_parameter("status_topic", status_topic);

        sub_steer_ = this->create_subscription<pacmod3_msgs::msg::SystemRptFloat>(steering_topic, 10, std::bind(&VehicleStatusPacmod::vehicleSteeringCallback, this, _1));
        sub_speed_ = this->create_subscription<pacmod3_msgs::msg::VehicleSpeedRpt>(speed_topic, 10, std::bind(&VehicleStatusPacmod::vehicleSpeedCallback, this, _1));
        status_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(status_topic, 1);
        RCLCPP_INFO_STREAM(this->get_logger(), "Node started: " << this->get_name() << " with steering topic: " << steering_topic << " and speed topic: " << speed_topic << " and status topic: " << status_topic);
    }

private:
  
    // Callback for steering wheel messages
    void vehicleSteeringCallback(const pacmod3_msgs::msg::SystemRptFloat &steer_msg)
    {
        steering_angle = steer_msg.output; // / 14.8;
    }

    // Callback for vehicle speed messages
    void vehicleSpeedCallback(const pacmod3_msgs::msg::VehicleSpeedRpt &speed_msg)
    {
        vehicle_speed_mps = speed_msg.vehicle_speed;
        geometry_msgs::msg::TwistStamped speed_kmph;
        speed_kmph.header.stamp = speed_msg.header.stamp;
        speed_kmph.twist.linear.x = vehicle_speed_mps;
        speed_kmph.twist.angular.z = steering_angle;
        status_pub_->publish(speed_kmph);
    }


   
    rclcpp::Subscription<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr sub_steer_;
    rclcpp::Subscription<pacmod3_msgs::msg::VehicleSpeedRpt>::SharedPtr sub_speed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr status_pub_;
    std::string steering_topic, speed_topic, status_topic;
    double steering_angle, vehicle_speed_mps;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleStatusPacmod>());
    rclcpp::shutdown();
    return 0;
}
