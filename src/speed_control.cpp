#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <pacmod3_msgs/msg/vehicle_speed_rpt.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include "geometry_msgs/msg/twist.hpp"

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_string_pub;
rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr accel_pub;
rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr brake_pub;
rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr steer_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub;


double vehicle_speed_actual, vehicle_speed_reference, vehicle_steering_reference;
double speed_diff, speed_diff_prev;
double current_time, prev_time, dt;
std_msgs::msg::String status_string_msg;
std_msgs::msg::Bool enable_msg;
std::string lx_namespace;
pacmod3_msgs::msg::SystemCmdFloat accel_command;
pacmod3_msgs::msg::SystemCmdFloat brake_command;
pacmod3_msgs::msg::SteeringCmd steer_command;
float p_gain_accel, i_gain_accel, d_gain_accel;
float p_gain_brake, i_gain_brake, d_gain_brake;
double p_out_accel, i_out_accel, d_out_accel = 0.0;
double p_out_brake, i_out_brake, d_out_brake = 0.0;
double const _max_i_accel = 1.0, _max_i_brake = 1.5; // anti windup constants TODO: check valid params 
double t_integ_accel = 0.0, t_integ_brake = 0.0; // anti windup
double t_derivative_accel, t_derivative_brake;
double accel_command_prev = 0.0, brake_command_prev = 0.0;
bool autonom_status_changed = true;
bool first_run = true;
bool control_state = true; // control_state: acceleration(true) / deceleration(false) state (necessary for hysteresis to avoid fluctuating speed)


// Functions
void speedCurrentCallback(const pacmod3_msgs::msg::VehicleSpeedRpt &speed_msg);
void speedReferenceCallback(const geometry_msgs::msg::Twist &ref_msg);

int main(int argc, char * argv[])
{

/*
 Subscribers:
    /pacmod/enabled: std_msgs/msg/Bool
    /pacmod/hazard_lights_rpt: pacmod3_msgs/msg/SystemRptBool
    /pacmod/headlight_rpt: pacmod3_msgs/msg/SystemRptInt
    /pacmod/horn_rpt: pacmod3_msgs/msg/SystemRptBool
    /pacmod/vehicle_speed_rpt: pacmod3_msgs/msg/VehicleSpeedRpt
    /pacmod/wiper_rpt: pacmod3_msgs/msg/SystemRptInt
  Publishers:
    /pacmod/accel_cmd: pacmod3_msgs/msg/SystemCmdFloat
    /pacmod/brake_cmd: pacmod3_msgs/msg/SystemCmdFloat
    /pacmod/hazard_lights_cmd: pacmod3_msgs/msg/SystemCmdBool
    /pacmod/headlight_cmd: pacmod3_msgs/msg/SystemCmdInt
    /pacmod/horn_cmd: pacmod3_msgs/msg/SystemCmdBool
    /pacmod/shift_cmd: pacmod3_msgs/msg/SystemCmdInt
    /pacmod/steering_cmd: pacmod3_msgs/msg/SteeringCmd
    /pacmod/turn_cmd: pacmod3_msgs/msg/SystemCmdInt
    /pacmod/wiper_cmd: pacmod3_msgs/msg/SystemCmdInt
*/

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("speed_control_node");

  node->declare_parameter<std::string>("lx_namespace", "");
  node->get_parameter("lx_namespace", lx_namespace);
  auto sub_current_speed = node->create_subscription<pacmod3_msgs::msg::VehicleSpeedRpt>(lx_namespace + "/pacmod/vehicle_speed_rpt", 10, speedCurrentCallback);
  auto sub_reference_speed = node->create_subscription<geometry_msgs::msg::Twist>(lx_namespace + "/cmd_vel", 10, speedReferenceCallback);

  accel_pub = node->create_publisher<pacmod3_msgs::msg::SystemCmdFloat>(lx_namespace + "/pacmod/accel_cmd", 10);
  brake_pub = node->create_publisher<pacmod3_msgs::msg::SystemCmdFloat>(lx_namespace + "/pacmod/brake_cmd", 10);
  steer_pub = node->create_publisher<pacmod3_msgs::msg::SteeringCmd>(lx_namespace + "/pacmod/steering_cmd", 10);
  enable_pub = node->create_publisher<std_msgs::msg::Bool>(lx_namespace + "/pacmod/enable", 10);
  status_string_pub = node->create_publisher<std_msgs::msg::String>(lx_namespace + "/control_status", 10);

  node->declare_parameter<float>("p_gain_accel", 0.29);
  node->declare_parameter<float>("i_gain_accel", 0.035);
  node->declare_parameter<float>("d_gain_accel", 0.0);
  node->declare_parameter<float>("p_gain_brake", 0.15);
  node->declare_parameter<float>("i_gain_brake", 0.038);
  node->declare_parameter<float>("d_gain_brake", 0.0);

  node->get_parameter("p_gain_accel", p_gain_accel);
  node->get_parameter("i_gain_accel", i_gain_accel);
  node->get_parameter("d_gain_accel", d_gain_accel);
  node->get_parameter("p_gain_brake", p_gain_brake);
  node->get_parameter("i_gain_brake", i_gain_brake);
  node->get_parameter("d_gain_brake", d_gain_brake);

  RCLCPP_INFO_STREAM(node->get_logger(), "Starting longitudinal (speed) control. Namespace: " << lx_namespace );
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
  //RCLCPP_INFO_STREAM(node->get_logger(), " diff km/h: " << speed_diff * 3.6);
  // here a hysteresis is applied to avoid fluctuating behaviour at constant speed
  // the bandwith of the hysteresis: e.g. 0.15 m/s (0.54 km/h)
  // in this if statement control_state is in acceleration
  if( (speed_diff > -0.15 && control_state) || (speed_diff > 0.15) ){
      control_state = true;    
      //RCLCPP_INFO_STREAM(node->get_logger(), "accelerate");
      p_out_accel = speed_diff * p_gain_accel;
      t_integ_accel += speed_diff * dt;
    i_out_accel = t_integ_accel * i_gain_accel;
    t_derivative_accel = (speed_diff - speed_diff_prev) / dt;
    d_out_accel = d_gain_accel * t_derivative_accel;
    double acclel_command_raw = p_out_accel + i_out_accel + d_out_accel; // P+I+D
    if (acclel_command_raw > _max_i_accel) // anti-windup limit 
    {
        t_integ_accel = acclel_command_raw - p_out_accel - d_out_accel;
        acclel_command_raw = _max_i_accel;
    }
    accel_command.command = acclel_command_raw;
    brake_command.command = 0.0;
    // gradient limit
    if((accel_command.command - accel_command_prev) / dt > 0.4 && dt > 0.0){
        accel_command.command = accel_command_prev + 0.4 * dt;
    }
    else if((accel_command.command - accel_command_prev) / dt < -1.6 && dt > 0.0){
        accel_command.command = accel_command_prev - 1.6 * dt;
    } 
  }
  // hysteresis to avoid fluctuating behaviour at constant speeds
  // in this if statement control_state is in deceleration (brake)
  else if( (speed_diff < 0.15 && !control_state) || (speed_diff < -0.15) ){
      control_state = false; // brake state
      //RCLCPP_INFO_STREAM(node->get_logger(), "brake");
      p_out_brake = speed_diff * p_gain_brake;
      t_integ_brake += speed_diff * dt;

      i_out_brake = t_integ_brake * i_gain_brake;
      t_derivative_brake = (speed_diff - speed_diff_prev) / dt;
      d_out_brake = d_gain_brake * t_derivative_brake;
      double brake_command_raw = p_out_brake + i_out_brake + d_out_brake; // P+I+D
      if (brake_command_raw > _max_i_brake) // anti-windup limit 
      {
          t_integ_brake = brake_command_raw - p_out_brake - d_out_brake;
          brake_command_raw = _max_i_brake;
      }
      brake_command.command = -1.0 * brake_command_raw;
      accel_command.command = 0.0;
      // gradient limit
      if((brake_command.command - brake_command_prev) / dt > 0.8 && dt > 0.0){
          brake_command.command = brake_command_prev + 0.8 * dt;
      }
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
    //RCLCPP_INFO_STREAM(node->get_logger(), "dt: " << dt);
  }
  {
    // /lexus3/pacmod/parsed_tx/vehicle_speed_rpt is assumed 30 Hz
    dt = 0.033333;
  }
  if(first_run){
    first_run = false;
    status_string_msg.data = "longitudinal_control_started";
  }
  enable_msg.data = true;
  accel_pub->publish(accel_command);
  brake_pub->publish(brake_command);
  steer_pub->publish(steer_command);
  enable_pub->publish(enable_msg);
  status_string_msg.data = control_state ? "accel" : "brake";
  if(status_string_msg.data.compare(""))
    status_string_pub->publish(status_string_msg);
  status_string_msg.data = "";
  prev_time = current_time;
  accel_command_prev = accel_command.command;
  brake_command_prev = brake_command.command;  
  speed_diff_prev = speed_diff;
}

void speedReferenceCallback(const geometry_msgs::msg::Twist &ref_msg){
    vehicle_speed_reference = ref_msg.linear.x;
    vehicle_steering_reference = ref_msg.angular.z;
}