/* header guard */
#ifndef _PCL_MERGER_HPP_
#define _PCL_MERGER_HPP_

// put includes here for the cpp file
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp> // comp node

#include <sensor_msgs/msg/point_cloud2.hpp>

// #include <tf2/utils.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/crop_box.h>

// put function signatures here


#endif // _PCL_MERGER_HPP_