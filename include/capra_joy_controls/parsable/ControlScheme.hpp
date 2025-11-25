#ifndef CAPRA_JOY_CONTROLS__PARSABLE__CONTROL_SCHEME_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__CONTROL_SCHEME_HPP_

#include "capra_joy_controls/parsable/Action.hpp"

namespace capra_joy_controls::parsable {

struct ControlScheme : YAMLParsable, RunnableAction {
    std::string name{};
    std::vector<Action> actions{};

    ControlScheme() = default;
    ControlScheme(const std::string& name, const std::vector<Action> actions = {})
    : name{name}, actions{actions} {}
    ControlScheme(const YAML::Node& node) { parse_from(node); }

    void parse_from(const YAML::Node& node) override;
    void init(rclcpp::Node::SharedPtr node) override;
    void run(const JoyContext& context) override;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__CONTROL_SCHEME_HPP_