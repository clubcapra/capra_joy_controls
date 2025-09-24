#ifndef CAPRA_JOY_CONTROLS__TYPES_HPP_
#define CAPRA_JOY_CONTROLS__TYPES_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace capra_joy_controls {

using btn_id = int;
using axis_id = int;

using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;
using Bool = std_msgs::msg::Bool;

} // capra_joy_controls

#endif // CAPRA_JOY_CONTROLS__TYPES_HPP_