#ifndef CAPRA_JOY_CONTROLS__YAML__EXCEPTIONS_HPP_
#define CAPRA_JOY_CONTROLS__YAML__EXCEPTIONS_HPP_

#include <yaml-cpp/yaml.h>
#include <typeinfo>

#include "capra_joy_controls/utils.hpp"

#define THROW(exception, ...) throw exception(__FILE__, __LINE__, __VA_ARGS__)


namespace capra_joy_controls::yaml {

class YAMLParseException : public std::runtime_error {
public:
    explicit YAMLParseException(const char* file, 
        int line, 
        const YAML::Node& node, 
        const std::string& what, 
        const std::runtime_error* inner = nullptr) 
    : std::runtime_error(
        (std::ostringstream() << file << ':' << line << 
        " - Error parsing yaml at " << mark2str(node.Mark()) << ':' << 
        what << (inner ? std::string("\nFrom inner:") + inner->what() : std::string())).str()
    ) {}
private:
    std::string mark2str(const YAML::Mark& mark) {
        if (mark.is_null()) return "";
        std::ostringstream oss;
        oss << "line:" << mark.line << " col:" << mark.column;
        return oss.str();
    }
};

class YAMLUndefinedException : public YAMLParseException {
public:
    explicit YAMLUndefinedException(const char* file, 
        int line, 
        const YAML::Node& node,
        const std::runtime_error* inner = nullptr)
    : YAMLParseException(file, line, node,
        "Node '" + node.Tag() + "' is undefined",
        inner
    ) {}

    explicit YAMLUndefinedException(const char* file, 
        int line,
        const YAML::Node& parent,
        const YAML::Node& node,
        const std::runtime_error* inner = nullptr)
    : YAMLParseException(file, line, node,
        "Node '" + node.Tag() + "' is undefined for object '" + parent.Tag() + "'",
        inner
    ) {}
};

class YAMLNullException : public YAMLParseException {
public:
    explicit YAMLNullException(const char* file, 
        int line,
        const YAML::Node& node,
        const std::runtime_error* inner = nullptr)
    : YAMLParseException(file, line, node,
        "Node '" + node.Tag() + "' is null",
        inner
    ) {}

    explicit YAMLNullException(const char* file, 
        int line,
        const YAML::Node& parent,
        const YAML::Node& node,
        const std::runtime_error* inner = nullptr)
    : YAMLParseException(file, line, node,
        "Node '" + node.Tag() + "' is null for object '" + parent.Tag() + "'",
        inner
    ) {}
};

template <typename TEnum>
class YAMLInvalidEnumValue : public YAMLParseException {
public:
    explicit YAMLInvalidEnumValue(const char* file, 
        int line,
        const YAML::Node& node,
        const std::string& entry,
        const std::vector<std::string>& allowedValues, 
        const std::runtime_error* inner = nullptr) 
    : YAMLParseException(file, line, node,
        "Invalid enum value '" + entry +
        "' for enum '" + typeid(TEnum).name() +
        "'. Allowed values: " + join(allowedValues, ", "),
        inner
    ) {}
};

class YAMLInvalidNodeType : public YAMLParseException {
public:
    explicit YAMLInvalidNodeType(const char* file, 
        int line,
        const YAML::Node& node,
        const YAML::NodeType::value& expected,
        const std::runtime_error* inner = nullptr) 
    : YAMLParseException(file, line, node,
        "Invalid node type '" + type2str(node.Type()) + "' expected '" + type2str(expected) + "'",
        inner
    ) {}

    explicit YAMLInvalidNodeType(const char* file, 
        int line,
        const YAML::Node& node,
        const std::vector<YAML::NodeType::value>& expected,
        const std::runtime_error* inner = nullptr) 
    : YAMLParseException(file, line, node,
        "Invalid node type '" + type2str(node.Type()) + "' expected '" + type2str(expected) + "'",
        inner
    ) {}
private:
    std::string type2str(const YAML::NodeType::value& type) {
        switch (type) {
            case YAML::NodeType::Map:
                return "Map";
            case YAML::NodeType::Scalar:
                return "Scalar";
            case YAML::NodeType::Sequence:
                return "Sequence";
            case YAML::NodeType::Null:
                return "Null";
            case YAML::NodeType::Undefined:
                return "Undefined";
        }
    }
    std::string type2str(const std::vector<YAML::NodeType::value>& types) {
        std::vector<std::string> strs;
        for (auto type : types) {
            strs.emplace_back(type2str(type));
        }
        return join(strs, ", ");
    }
};

template <typename TField>
class YAMLInvalidFieldType : public YAMLParseException {
public:
    explicit YAMLInvalidFieldType(const char* file, 
        int line,
        const YAML::Node& node,
        const std::runtime_error* inner = nullptr) 
    : YAMLParseException(file, line, node,
        "Invalid value type for field '" + node.Tag() + "'. Expected '" + typeid(TField).name() + "'",
        inner
    ) {}
};

template <typename TValue>
class YAMLInvalidValue : public YAMLParseException {
public:
    explicit YAMLInvalidValue(const char* file, 
        int line,
        const YAML::Node& node,
        const TValue& value,
        const std::vector<TValue>& allowedValues,
        const std::runtime_error* inner = nullptr)
    : YAMLParseException(file, line, node,
        "Invalid value for field '" + node.Tag() + "' expected one of [" + join(allowedValues, ", ") + "' got '" + node.Scalar() + "'",
        inner
    ) {}
    explicit YAMLInvalidValue(const char* file, 
        int line,
        const YAML::Node& node,
        const TValue& value,
        const TValue& min,
        const TValue& max,
        const std::runtime_error* inner = nullptr)
    : YAMLParseException(file, line, node,
        (std::ostringstream() << "Invalid value for field '" << node.Tag() << 
        "' expected a value between '" << min << "' and '" << max << 
        "' got '" << node.Scalar() << "'").str(),
        inner
    ) {}
};



} // capra_joy_controls::yaml

#endif // CAPRA_JOY_CONTROLS__YAML__EXCEPTIONS_HPP_