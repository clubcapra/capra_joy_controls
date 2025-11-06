#ifndef CAPRA_JOY_CONTROLS__PARSABLE__TRIGGER_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__TRIGGER_HPP_

#include "capra_joy_controls/parsable/YAMLParsable.hpp"

namespace capra_joy_controls::parsable {

struct Value;

struct JoyTrigger {
    virtual bool read(const JoyContext& context) = 0;
};

/**
 * @brief Trigger
 * @code {.yaml}
```yaml
## Types:
# On/Off or 0/1
parent:
    trig: 0
    trig: 1

# Button
parent:
    trig:
        button: 5
    
    # Equivalent to
    trig:
        button:
            id: 5 # Joy button index                    | Required
            event: "held" # Activation state            | Optional (default:held) [options:held, on_press, on_release, not_held]
  
# Axis range
parent:
    trig:
        axis_range:
            id: 5 # Joy axis index                      | Required
            event: "inside" # Activation state          | Optional (default:inside) [options:inside, on_enter, on_exit, outside]
            min: 0.5 # Minimum value                    | Optional unless max not defined (default:undefined)
            max: 1.0 # Maximum value                    | Optional unless min not defined (default:undefined)
  
# Conditional
parent:
    trig:
        condition:
            conditions: # List of triggers              | Required
                - button: 5
                - 0
            while_true: 1 # Trigger                     | Optional (default:1)
            while_false: 0 # Trigger                    | Optional (default:0)
  
    # As mentioned earlier, this would also be valid
    while_true:
        axis_range:
            id: 5
            min: 0.5
```
* @endcode
* 
*/
struct Trigger : YAMLParsable, JoyTrigger {
    // Always off
    struct Off : YAMLParsable, JoyTrigger {
        Off() = default;
        explicit Off(const YAML::Node& node) { parse_from(node); }
        void parse_from(const YAML::Node& node) override { }
        bool read(const JoyContext& context) override;
    };
    // Always on
    struct On : YAMLParsable, JoyTrigger {
        On() = default;
        explicit On(const YAML::Node& node) { parse_from(node); }
        void parse_from(const YAML::Node& node) override { }
        bool read(const JoyContext& context) override;
    };
    // Button binding
    struct Button : YAMLParsable, JoyTrigger {
        YAML_ENUM(ButtonEventType,
            held,
            on_press,
            on_release,
            not_held
        )

        btn_id id = -1;
        ButtonEventType event = ButtonEventType::held;

        Button() = default;
        Button(const btn_id& id = -1, const ButtonEventType& event = ButtonEventType::held)
        : id{id}, event{event} {}
        explicit Button(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
        bool read(const JoyContext& context) override;
    };
    // Axis range binding
    struct AxisRange : YAMLParsable, JoyTrigger {
        YAML_ENUM(AxisRangeEventType,
            inside,
            on_enter,
            on_exit,
            outside
        )

        axis_id id = -1;
        AxisRangeEventType event = AxisRangeEventType::inside;
        float min = NAN;
        float max = NAN;

        AxisRange() = default;
        AxisRange(const axis_id& id, const AxisRangeEventType& event, const float& min = NAN, const float& max = NAN)
        : id{id}, event{event}, min{min}, max{max} {}
        explicit AxisRange(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
        bool read(const JoyContext& context) override;
    };
    // Conditional trigger
    struct Condition : YAMLParsable, JoyTrigger {
        YAML_ENUM(ConditionOperator,
            _and,
            _nand,
            _or,
            _nor,
            _xor,
            _nxor
        )
        std::vector<Trigger> conditions = {};
        ConditionOperator oper = ConditionOperator::_and;
        Trigger *while_true = nullptr;
        Trigger *while_false = nullptr;

        Condition() = default;
        Condition(const std::vector<Trigger>& conditions, const ConditionOperator& oper = ConditionOperator::_and, const Trigger& while_true = Trigger::On(), const Trigger& while_false = Trigger::Off()) 
        : conditions(conditions), while_true(new Trigger(while_true)), while_false(new Trigger(while_false)) {}
        Condition(const Condition& other) {
            conditions = other.conditions;
            oper = other.oper;
            if (other.while_true) {
                while_true = new Trigger(*other.while_true);
            }
            if (other.while_false) {
                while_false = new Trigger(*other.while_false);
            }
        }
        explicit Condition(const YAML::Node& node) { parse_from(node); }

        void set_while_true(Trigger* trigger);
        void set_while_false(Trigger* trigger);

        void parse_from(const YAML::Node& node) override;
        bool read(const JoyContext& context) override;
    
        ~Condition() {
            if (while_true) {
                delete while_true;
                while_true = nullptr;
            }
            if (while_false) {
                delete while_false;
                while_false = nullptr;
            }
        }
    };

    // Define trigger types and parsing methods
    YAML_ENUM(TriggerType, 
        off,
        on,
        button,
        axis_range,
        condition
    )

    // Fields
    std::variant<
        Off,
        On,
        Button,
        AxisRange,
        Condition
    > value{Off()};

    TriggerType type() const { return (TriggerType)value.index(); }
    GET_VARIANT(off, Off, value)
    GET_VARIANT(on, On, value)
    GET_VARIANT(button, Button, value)
    GET_VARIANT(axis_range, AxisRange, value)
    GET_VARIANT(condition, Condition, value)

    // Creation methods
    static Trigger create_off(const Off& _off = Off()) { return {_off}; }
    static Trigger create_on(const On& _on = On()) { return {_on}; }
    static Trigger create_button(const Button& button) { return {button}; }
    static Trigger create_button(const btn_id& id, const Button::ButtonEventType& event = Button::held) { return Button(id, event); }
    static Trigger create_axis_range(const AxisRange& trigger) { return {trigger}; }
    static Trigger create_axis_range(const axis_id& id, const AxisRange::AxisRangeEventType& event = AxisRange::inside, const float& min = NAN, const float& max = NAN)
    { return create_axis_range(AxisRange(id, event, min, max)); }
    static Trigger create_condition(const Condition& condition) { return {condition}; }
    static Trigger create_condition(const std::vector<Trigger>& conditions, const Condition::ConditionOperator& oper = Condition::ConditionOperator::_and, const Trigger& while_true = On(), const Trigger& while_false = Off())
    { return create_condition(Condition{conditions, oper, while_true, while_false}); }

    // Assignement operators
    Trigger& operator=(const bool& state) {
        if (state) { value = On(); }
        else { value = Off(); }
        return *this;
    }
    Trigger& operator=(const Off& _off) {
        value = _off;
        return *this;
    }
    Trigger& operator=(const On& _on) {
        value = _on;
        return *this;
    }
    Trigger& operator=(const Button& _button) {
        value = _button;
        return *this;
    }
    Trigger& operator=(const AxisRange& _axis_range) {
        value = _axis_range;
        return *this;
    }
    Trigger& operator=(const Condition& _condition) {
        value = _condition;
        return *this;
    }

    // Constructors
    Trigger() = default;
    Trigger(const bool& state) {
        if (state) { value = On(); }
        else { value = Off(); }
    }
    Trigger(const Off& _off) : value{_off} {}
    Trigger(const On& _on) : value{_on} {}
    Trigger(const Button& _button) : value{_button} {}
    Trigger(const AxisRange& _axis_range) : value{_axis_range} {}
    Trigger(const Condition& _condition) : value{_condition} {}
    explicit Trigger(const YAML::Node& node) { parse_from(node); }

    void parse_from(const YAML::Node& node) override;
    bool read(const JoyContext& context) override;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__TRIGGER_HPP_