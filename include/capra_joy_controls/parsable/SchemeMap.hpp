#ifndef CAPRA_JOY_CONTROLS__PARSABLE__SCHEME_MAP_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__SCHEME_MAP_HPP_

#include "capra_joy_controls/parsable/ControlScheme.hpp"

namespace capra_joy_controls::parsable {

struct SchemeMap : YAMLParsable, RunnableAction {
    std::vector<ControlScheme> controlSchemes{};

    SchemeMap() = default;
    SchemeMap(const std::vector<ControlScheme> controlSchemes)
    : controlSchemes{controlSchemes} {}
    SchemeMap(const YAML::Node& node) { parse_from(node); }

    void parse_from(const YAML::Node& node) override;
    void init(rclcpp::Node::SharedPtr node) override;
    void run(const JoyContext& context) override;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__SCHEME_MAP_HPP_