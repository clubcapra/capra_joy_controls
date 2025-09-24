#ifndef CAPRA_JOY_CONTROLS__PARSABLE__YAML_PARSABLE_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__YAML_PARSABLE_HPP_

#include "capra_joy_controls/yaml.hpp"
#include "capra_joy_controls/types.hpp"

#define GET_VARIANT(id, valueType, variant) std::optional<valueType> get_##id() const { if (type() == id) return std::get<valueType>(variant); }

namespace capra_joy_controls::parsable {

struct YAMLParsable {
    YAMLParsable() = default;
    virtual ~YAMLParsable() = default;
    virtual void parse_from(const YAML::Node& node) = 0;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__YAML_PARSABLE_HPP_