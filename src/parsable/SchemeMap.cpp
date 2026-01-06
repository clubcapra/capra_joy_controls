#include "capra_joy_controls/parsable/SchemeMap.hpp"

namespace capra_joy_controls::parsable
{

void SchemeMap::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);

    for (auto ncontrolScheme : node) {
        auto c = ncontrolScheme.second;
        c.SetTag(ncontrolScheme.first.Scalar());
        controlSchemes.emplace_back(c);
    }
}

void SchemeMap::init(ContainerNode& node)
{
    RCLCPP_INFO(node.get_logger(), "Initializing scheme map");
    for (auto s : controlSchemes) {
        s.init(node);
    }
}

void SchemeMap::run(const JoyContext &context)
{
    // RCLCPP_INFO(rclcpp::get_logger("joy_controls"), "Running scheme map");
    int i = 0;
    for (auto s : controlSchemes) {
        // RCLCPP_INFO(rclcpp::get_logger("joy_controls"), "Running scheme map[%d]", i++);
        s.run(context);
    }
}

} // namespace capra_joy_controls::parsable
