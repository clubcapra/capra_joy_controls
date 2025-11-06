#include "capra_joy_controls/parsable/Trigger.hpp"
#include "capra_joy_controls/parsable/Value.hpp"

namespace capra_joy_controls::parsable {

bool Trigger::Off::read(const JoyContext &context)
{
    return false;
}

bool Trigger::On::read(const JoyContext &context)
{
    return true;
}

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

bool Trigger::Button::read(const JoyContext &context)
{
    switch (event)
    {
    case ButtonEventType::held:
        return context.button(id);
    case ButtonEventType::not_held:
        return !context.button(id);
    case ButtonEventType::on_press:
        return context.rising(id);
    case ButtonEventType::on_release:
        return context.falling(id);
    default:
        return false;
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

bool Trigger::AxisRange::read(const JoyContext &context)
{
    switch (event)
    {
    case AxisRangeEventType::inside:
        return context.inside(id, min, max);
    case AxisRangeEventType::outside:
        return !context.inside(id, min, max);
    case AxisRangeEventType::on_enter:
        return context.entered(id, min, max);
    case AxisRangeEventType::on_exit:
        return context.exited(id, min, max);
    default:
        return 0.f;
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

    if (auto noper = node["operator"]) {
        expect_node_type(noper, YAML::NodeType::Scalar);
        oper = parse_enum_ConditionOperator(noper, "_" + noper.Scalar());
    }

    if (auto nwhile_true = node["while_true"]) {
        set_while_true(new Trigger(nwhile_true));
    }

    if (auto nwhile_false = node["while_false"]) {
        set_while_false(new Trigger(nwhile_false));
    }
}

bool Trigger::Condition::read(const JoyContext &context)
{
    auto eval = [&](){
        switch (oper)
        {
        case ConditionOperator::_and:
            for (auto t : conditions) {
                if (!t.read(context)) return false;
            }
            return true;
        case ConditionOperator::_nand:
            for (auto t : conditions) {
                if (!t.read(context)) return true;
            }
            return false;
        case ConditionOperator::_or:
            for (auto t : conditions) {
                if (t.read(context)) return true;
            }
            return false;
        case ConditionOperator::_nor:
            for (auto t : conditions) {
                if (t.read(context)) return false;
            }
            return true;
        case ConditionOperator::_xor:
            {
                int count = 0;
                for (auto t : conditions) {
                    if (t.read(context)) ++count;
                }
                return count == 1;
            }
        case ConditionOperator::_nxor:
            {
                int count = 0;
                for (auto t : conditions) {
                    if (t.read(context)) ++count;
                }
                return count != 1;
            }
        default:
            return false;
        }
    };
    return eval() ? (while_true && while_true->read(context)) : (while_false && while_false->read(context));
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

bool Trigger::read(const JoyContext &context)
{
    switch (type()) {
        case TriggerType::button:
            return get_button()->read(context);
        case TriggerType::axis_range:
            return get_axis_range()->read(context);
        case TriggerType::condition:
            return get_condition()->read(context);
        case TriggerType::on:
            return get_on()->read(context);
        case TriggerType::off:
            return get_off()->read(context);
        default:
            return false;
    }
}



}
