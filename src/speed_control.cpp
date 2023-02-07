#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp> // TODO: ros1 version was SteerSystemCmd.h
#include "geometry_msgs/msg/twist.hpp"

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_string_pub;
rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr accel_pub;
rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr brake_pub;
rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr steer_pub; // TODO: ros1 was SteerSystemCmd

double vehicle_speed_actual, vehicle_speed_reference, vehicle_steering_reference;
double speed_diff, speed_diff_kmh, speed_diff_prev;
double current_time, prev_time, dt;
std_msgs::msg::String status_string_msg;
pacmod3_msgs::msg::SystemCmdFloat accel_command;
pacmod3_msgs::msg::SystemCmdFloat brake_command;
pacmod3_msgs::msg::SteeringCmd steer_command;
float p_gain_accel, i_gain_accel, d_gain_accel;
float p_gain_brake, i_gain_brake, d_gain_brake;
double p_out_accel, i_out_accel, d_out_accel = 0.0;
double p_out_brake, i_out_brake, d_out_brake = 0.0;
double const _max_i_accel = 1.0, _max_i_brake = 1.0; // anti windup constants TODO: check valid params 
double t_integ_accel, t_integ_brake; // anti windup
double t_derivative_accel, t_derivative_brake;
bool autonom_status_changed = true;
bool first_run = true, standstill = false;


// Functions
void speedCurrentCallback(const pacmod3_msgs::msg::VehicleSpeedRpt &speed_msg);
void speedReferenceCallback(const geometry_msgs::msg::Twist &ref_msg);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("speed_control_node");
  auto sub_current_speed = node->create_subscription<pacmod3_msgs::msg::VehicleSpeedRpt>("/pacmod/parsed_tx/vehicle_speed_rpt", 10, speedCurrentCallback);
  auto sub_reference_speed = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, speedReferenceCallback);

  accel_pub = node->create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/as_rx/accel_cmd", 10);
  brake_pub = node->create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/as_rx/brake_cmd", 10);
  steer_pub = node->create_publisher<pacmod3_msgs::msg::SteeringCmd>("/pacmod/as_rx/steer_cmd", 10);
  status_string_pub = node->create_publisher<std_msgs::msg::String>("control_status", 10);

  node->declare_parameter<float>("p_gain_accel", 15.0);
  node->declare_parameter<float>("i_gain_accel", 1.0);
  node->declare_parameter<float>("d_gain_accel", 0.0);
  node->declare_parameter<float>("p_gain_brake", 9.0);
  node->declare_parameter<float>("i_gain_brake", 1.0);
  node->declare_parameter<float>("d_gain_brake", 0.0);

  node->get_parameter("p_gain_accel", p_gain_accel);
  node->get_parameter("i_gain_accel", i_gain_accel);
  node->get_parameter("d_gain_accel", d_gain_accel);
  node->get_parameter("p_gain_brake", p_gain_brake);
  node->get_parameter("i_gain_brake", i_gain_brake);
  node->get_parameter("d_gain_brake", d_gain_brake);

  RCLCPP_INFO(node->get_logger(), "Starting lateral control...");
  RCLCPP_INFO_STREAM(node->get_logger(), "Accel PID: " <<  p_gain_accel << " | " << i_gain_accel << " | " << d_gain_accel);
  RCLCPP_INFO_STREAM(node->get_logger(), "Brake PID: " <<  p_gain_brake << " | " << i_gain_brake << " | " << d_gain_brake);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
  try {
    rclcpp::shutdown();
  } catch (std::exception& e) {
    // TODO: Log the exception and shutdown
  }
  return 0;
}

void speedCurrentCallback(const pacmod3_msgs::msg::VehicleSpeedRpt &speed_msg)
{
  current_time = (node->now()).seconds();
  vehicle_speed_actual = speed_msg.vehicle_speed;
  steer_command.command = vehicle_steering_reference;
  //RCLCPP_INFO_STREAM(node->get_logger(), "speed: " << speed_msg.vehicle_speed);
  speed_diff = vehicle_speed_reference - vehicle_speed_actual;
  speed_diff_kmh = speed_diff * 3.6;
  //RCLCPP_INFO_STREAM(node->get_logger(), " diff km/h: " << speed_diff_kmh);
  if(speed_diff > 0){
      //RCLCPP_INFO_STREAM(node->get_logger(), "accelerate");
      p_out_accel = speed_diff * p_gain_accel * dt;
      t_integ_accel += p_out_accel * dt;
      // Restrict to max Anti-Windup
      if( t_integ_accel > _max_i_accel)
          t_integ_accel = _max_i_accel;
    i_out_accel = t_integ_accel * i_gain_accel;
    t_derivative_accel = (speed_diff - speed_diff_prev) / dt;
    d_out_accel = d_gain_accel * t_derivative_accel;
     accel_command.command = p_out_accel + i_out_accel + d_out_accel;
     brake_command.command = 0.0;
  }
  else if(speed_diff < 0){
      //RCLCPP_INFO_STREAM(node->get_logger(), "brake");
      p_out_brake = speed_diff * p_gain_brake * dt;
      t_integ_brake += p_out_brake * dt;
      // Restrict to max Anti-Windup
      if( t_integ_brake > _max_i_brake)
          t_integ_brake = _max_i_brake;
      i_out_brake = t_integ_brake * i_gain_brake;
      t_derivative_brake = (speed_diff - speed_diff_prev) / dt;
      d_out_brake = d_gain_brake * t_derivative_brake;
      brake_command.command = -1.0 * (p_out_brake + i_out_brake + d_out_brake);
      accel_command.command = 0.0;
  }
  // TODO: add standstill ?
  if (standstill){
      brake_command.command = 0.2;
      accel_command.command = 0.0;
      status_string_msg.data = "standstill";
  }
  steer_command.rotation_rate = 3.3;
  accel_command.header.frame_id = "pacmod";
  accel_command.enable = true;
  brake_command.enable = true;
  steer_command.enable = true;
  if(autonom_status_changed){ 
    accel_command.clear_override = true;
    brake_command.clear_override = true;
    steer_command.clear_override = true;
    autonom_status_changed = false; // just once
    }
  else{
    accel_command.clear_override = false;
    brake_command.clear_override = false;
    steer_command.clear_override = false;
    }

  if(!first_run){
    dt = current_time - prev_time;
    status_string_msg.data = "lateral_control_started";
    //RCLCPP_INFO_STREAM(node->get_logger(), "dt: " << dt);
  }
  {
    // /lexus3/pacmod/parsed_tx/vehicle_speed_rpt is assumed 30 Hz
    dt = 0.033333;
  }
  if(first_run){
    first_run = false;
    status_string_msg.data = "lateral_control_started";
  }
  accel_pub->publish(accel_command);
  brake_pub->publish(brake_command);
  steer_pub->publish(steer_command);
  if(status_string_msg.data.compare(""))
    status_string_pub->publish(status_string_msg);
  status_string_msg.data = "";
  prev_time = current_time; 
  speed_diff_prev = speed_diff;
}

void speedReferenceCallback(const geometry_msgs::msg::Twist &ref_msg){
    vehicle_speed_reference = ref_msg.linear.x;
    vehicle_steering_reference = ref_msg.angular.z;
}