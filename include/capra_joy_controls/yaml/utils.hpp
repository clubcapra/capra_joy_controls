#ifndef CAPRA_JOY_CONTROLS__YAML__UTILS_HPP_
#define CAPRA_JOY_CONTROLS__YAML__UTILS_HPP_

#include "capra_joy_controls/yaml/exceptions.hpp"

namespace capra_joy_controls::yaml {

inline void expect_not_null(const YAML::Node& node) {
    if (node.IsNull()) THROW(YAMLUndefinedException, node);
}

inline void expect_not_null(const YAML::Node& parent, const YAML::Node& node) {
    if (node.IsNull()) THROW(YAMLUndefinedException, parent, node);
}

inline void expect_defined(const YAML::Node& node) {
    if (!node.IsDefined()) THROW(YAMLUndefinedException, node);
}

inline void expect_defined(const YAML::Node& parent, const YAML::Node& node) {
    if (!node.IsDefined()) THROW(YAMLUndefinedException, parent, node);
}

inline void expect_node_type(const YAML::Node& node, const YAML::NodeType::value& type) {
    if (node.Type() != type) THROW(YAMLInvalidNodeType, node, type);
}

inline void expect_node_type(const YAML::Node& node, const std::vector<YAML::NodeType::value>& types) {
    if (auto match = std::find(types.cbegin(), types.cend(), node.Type()); match == types.cend()) THROW(YAMLInvalidNodeType, node, types);
}

template <typename TValue>
inline TValue parse_value(const YAML::Node& node) {
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Scalar);
    try {
        return node.as<TValue>();
    } catch (const std::runtime_error& e) {
        THROW(YAMLInvalidFieldType<TValue>, node, &e);
    }
}

template <typename TEnum>
inline TEnum parse_any_enum(const YAML::Node& node, const std::string& value, const std::vector<std::string>& values) {
    for (int i = 0; i < values.size(); ++i) {
        if (value == values[i]) {
            return (TEnum)i;
        }
    }
    THROW(YAMLInvalidEnumValue<TEnum>, node, value, values);
}

template <typename TEnum>
inline TEnum parse_any_enum(const YAML::Node& node, const std::vector<std::string>& values) {
    expect_not_null(node);
    expect_node_type(node, YAML::NodeType::Scalar);
    return parse_any_enum<TEnum>(node, node.Scalar(), values);
}

#define YAML_ENUM(name, ...) \
enum name { \
    __VA_ARGS__ \
}; \
static name parse_enum_##name(const YAML::Node& node) { \
    return capra_joy_controls::yaml::parse_any_enum<name>(node, capra_joy_controls::split(", ", #__VA_ARGS__)); \
} \
static name parse_enum_##name(const YAML::Node& node, const std::string& value) { \
    return capra_joy_controls::yaml::parse_any_enum<name>(node, value, capra_joy_controls::split(", ", #__VA_ARGS__)); \
}

} // capra_joy_controls::yaml

#endif // CAPRA_JOY_CONTROLS__YAML__UTILS_HPP_