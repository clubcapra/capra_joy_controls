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

} // capra_joy_controls::parsable