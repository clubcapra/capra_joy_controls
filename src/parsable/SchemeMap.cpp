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

void SchemeMap::init(rclcpp::Node::SharedPtr node)
{
    RCLCPP_INFO(node->get_logger(), "Initializing scheme map");
    for (auto s : controlSchemes) {
        s.init(node);
    }
}

void SchemeMap::run(const JoyContext &context)
{
    for (auto s : controlSchemes) {
        s.run(context);
    }
}

} // namespace capra_joy_controls::parsable
