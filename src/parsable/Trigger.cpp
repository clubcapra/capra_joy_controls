#include "capra_joy_controls/parsable/Trigger.hpp"
#include "capra_joy_controls/parsable/Value.hpp"

namespace capra_joy_controls::parsable {

void Trigger::Button::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, {YAML::NodeType::Map, YAML::NodeType::Scalar});

    if (node.IsScalar()) {
        /* Abridged version
        trig:
            button: <id>
        */
        int v = parse_value<int>(node);
        id = v;
        return;
    }
    expect_node_type(node, YAML::NodeType::Map);

    /* Full version
    trig:
        button:
            id: <id>
            event: <event> (optional)
    */
    auto nid = node["id"];
    expect_defined(nid);
    id = parse_value<int>(nid);
    if (auto nevent = node["event"]) {
        event = parse_enum_ButtonEventType(nevent);
    }
}

void Trigger::AxisRange::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Map);

    /* Full version
    trig:
        axis_range:
            id: <id>
            event: <event> (optional)
            min: <min> (Optional unless max not defined)
            max: <max> (Optional unless min not defined)
    */
    auto nid = node["id"];
    expect_defined(nid); // Required
    id = parse_value<int>(nid);
    if (auto nevent = node["event"]) { // Optional
        event = parse_enum_AxisRangeEventType(nevent);
    }
    if (auto nmin = node["min"]) {
        min = parse_value<float>(nmin);
    }
    if (auto nmax = node["max"]) {
        max = parse_value<float>(nmax);
    }
    if (std::isnan(min) && std::isnan(max)) {
        THROW(YAMLParseException, node, "At least 'min' or 'max' need to be specified for axis_range");
    }
}

void Trigger::Condition::set_while_true(Trigger *trigger)
{
    if (while_true) {
        delete while_true;
    }
    while_true = trigger;
}

void Trigger::Condition::set_while_false(Trigger * trigger)
{
    if (while_false) {
        delete while_false;
    }
    while_false = trigger;
}

void Trigger::Condition::parse_from(const YAML::Node &node)
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
        set_while_true(new Trigger(nwhile_true));
    }

    if (auto nwhile_false = node["while_false"]) {
        set_while_false(new Trigger(nwhile_false));
    }
}
void Trigger::parse_from(const YAML::Node &node)
{
    expect_defined(node);
    expect_node_type(node, {YAML::NodeType::Map, YAML::NodeType::Scalar});

    if (node.IsScalar()) {
        /* Abridged version:
        parent:
            trig: 0
            trig: 1
        */
        int v = parse_value<int>(node);
        if (v == 0) {
            value = Off();
            return;
        }
        if (v == 1) {
            value = On();
            return;
        }
        THROW(YAMLInvalidValue, node, v, {0, 1});
    }
    expect_node_type(node, YAML::NodeType::Map);
    for (auto entry : node) {
        TriggerType type = parse_enum_TriggerType(entry.second, entry.first.Scalar());
        switch (type) {
            case TriggerType::button:
                value = Button(entry.second);
                return;
            case TriggerType::axis_range:
                value = AxisRange(entry.second);
                return;
            case TriggerType::condition:
                value = Condition(entry.second);
                return;
            case TriggerType::on:
            case TriggerType::off:
            default:
                break;
        }
        THROW(YAMLInvalidValue<std::string>, entry.second, entry.first.Scalar(), {"button", "axis_range", "condition"});
    }
    
}
}
