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

inline bool in_range(const float& v, const float& min = NAN, const float& max = NAN) {
    return !(std::isnan(v) || (!std::isnan(min) && v < min) || (!std::isnan(max) && v > max));
}

struct JoyContext {
    Joy::ConstSharedPtr last;
    Joy::ConstSharedPtr current;
    void next(Joy::ConstSharedPtr latest) {
        last = current;
        current = latest;
    }

    bool button(const btn_id& id) const {
        if (!current) return false;
        return current->buttons[id];
    }

    bool rising(const btn_id& id) const {
        if (!last || !current) return false;
        if (last->buttons[id] == -1 || current->buttons[id] == -1) return false;
        return !last->buttons[id] && current->buttons[id];
    }

    bool falling(const btn_id& id) const {
        if (!last || !current) return false;
        if (last->buttons[id] == -1 || current->buttons[id] == -1) return false;
        return last->buttons[id] && !current->buttons[id];
    }

    float axis(const axis_id& id) const {
        if (!current) return 0.f;
        return current->axes[id];
    }

    bool inside(const axis_id& id, const float& min = NAN, const float& max = NAN) const {
        if (!current) return false;
        return in_range(current->axes[id], min, max);
    }

    bool entered(const axis_id& id, const float& min = NAN, const float& max = NAN) const {
        if (!last || !current) return false;
        return !in_range(last->axes[id], min, max) && in_range(current->axes[id], min, max);
    }

    bool exited(const axis_id& id, const float& min = NAN, const float& max = NAN) const {
        if (!last || !current) return false;
        return in_range(last->axes[id], min, max) && !in_range(current->axes[id], min, max);
    }
};

} // capra_joy_controls

#endif // CAPRA_JOY_CONTROLS__TYPES_HPP_