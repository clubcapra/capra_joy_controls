#include "capra_joy_controls/parsable/Value.hpp"
#include "capra_joy_controls/parsable/Trigger.hpp"

namespace capra_joy_controls::parsable {

void Value::Constant::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Scalar);
    value = parse_value<float>(node);
}

void Value::Axis::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    
    auto nid = node["id"];
    expect_defined(nid); // Required
    id = parse_value<int>(nid);

    if (auto nmin = node["min"]) { // Optional
        min = parse_value<float>(nmin);
    }

    if (auto nmax = node["max"]) { // Optional
        max = parse_value<float>(nmax);
    }
}

void Value::Condition::set_while_true(Value *value)
{
    if (while_true) {
        delete while_true;
    }
    while_true = value;
}

void Value::Condition::set_while_false(Value * value)
{
    if (while_false) {
        delete while_false;
    }
    while_false = value;
}

void Value::Condition::parse_from(const YAML::Node& node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Map);

    /* Full version
    trig:
        condition:
            conditions: <trigger list> (Required)
            while_true: <trigger> (Optional)
            while_false: <trigger> (Optional)
    */
    auto nconditions = node["conditions"];
    expect_defined(nconditions); // Required
    expect_node_type(nconditions, YAML::NodeType::Sequence);
    for (auto ncondition : nconditions) {
        expect_defined(ncondition);
        conditions.push_back(Trigger(ncondition));
    }
    if (conditions.size() == 0) {
        THROW(YAMLParseException, node, "At least one condition is required");
    }

    if (auto nwhile_true = node["while_true"]) {
        set_while_true(new Value(nwhile_true));
    }

    if (auto nwhile_false = node["while_false"]) {
        set_while_false(new Value(nwhile_false));
    }
}

void Value::parse_from(const YAML::Node &node)
{
    expect_defined(node);
    expect_node_type(node, {YAML::NodeType::Map, YAML::NodeType::Scalar});

    if (node.IsScalar()) {
        /* Abridged version:
        parent:
            value: <constant>
        */
        value = Constant(parse_value<float>(node));
        return;
    }
    if (node.IsMap()) {
        for (auto entry : node) {
            ValueType type = parse_enum_ValueType(entry.second, entry.first.Scalar());
            switch (type) {
                case ValueType::constant:
                    value = Constant(entry.second);
                    return;
                case ValueType::axis:
                    value = Axis(entry.second);
                    return;
                case ValueType::condition:
                    value = Condition(entry.second);
                    return;
                default:
                    break;
            }
            THROW(YAMLInvalidValue<std::string>, entry.second, entry.first.Scalar(), {"button", "axis_range", "condition"});
        }
    }
}

}
