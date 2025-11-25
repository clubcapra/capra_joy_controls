#include "capra_joy_controls/parsable/ControlScheme.hpp"

namespace capra_joy_controls::parsable {

void ControlScheme::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, {YAML::NodeType::Map, YAML::NodeType::Sequence});

    name = node.Tag();
    for (auto entry : node) {
        actions.emplace_back(Action(node.Type() == YAML::NodeType::Map ? entry.second : entry));
    }
}

void ControlScheme::init(rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "Initializing control scheme %s", name);
    for (auto a : actions) {
        a.init(node);
    }
}

void ControlScheme::run(const JoyContext &context)
{
    for (auto a : actions) {
        a.run(context);
    }
}

} // capra_joy_controls::parsable