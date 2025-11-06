#ifndef CAPRA_JOY_CONTROLS__PARSABLE__VALUE_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__VALUE_HPP_

#include "capra_joy_controls/parsable/YAMLParsable.hpp"

namespace capra_joy_controls::parsable {

struct Trigger;

struct JoyValue {
    virtual float read(const JoyContext& context) = 0;
};

/**
 * @brief Value
 * @code {.yaml}
```yaml
## Value
# Constant
parent:
    value: 0.5

    # Equivalent to
    value:
        constant: 0.5
  
# Axis
parent:
    value:
        axis: 5 # Joy axis index

    # Equivalent to
    value:
        axis:
        id: 5 # Joy axis index              | Required
        min: -1.0 # Axis minimum            | Optional (default:-1)
        max: 1.0 # Axis maximum             | Optional (default:1)

# Condition
parent:
    value:
        condition:
        conditions: # List of triggers      | Required
            - button: 5
        operator: "and" # Operator          | Optional (default:and) [options:and, nand, or, nor, xor, nxor]
        while_true: 1 # Value               | Optional (default:1)
        while_false: 0 # Value              | Optional (default:0)
```
 * @endcode
 * 
 * 
 */
struct Value : YAMLParsable, JoyValue {
    // Constant value
    struct Constant : YAMLParsable, JoyValue {
        float value = 0;

        Constant() = default;
        Constant(const float& value) : value{value} {}
        explicit Constant(const YAML::Node& node) { parse_from(node); }
        void parse_from(const YAML::Node& node) override;
        float read(const JoyContext& context) override;
    };
    struct Axis : YAMLParsable, JoyValue {
        axis_id id = -1;
        float min = NAN;
        float max = NAN;
        Axis() = default;
        Axis(const axis_id& id, const float& min = NAN, const float& max = NAN) 
        : id{id}, min{min}, max{max} {}
        explicit Axis(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
        float read(const JoyContext& context) override;
    };
    struct Condition : YAMLParsable, JoyValue {
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
        Value* while_true = nullptr;
        Value* while_false = nullptr;

        Condition() = default;
        Condition(const std::vector<Trigger>& conditions, const ConditionOperator& oper = ConditionOperator::_and, const Value& while_true = Value::Constant(1), const Value& while_false = Value::Constant(0)) 
        : conditions(conditions), oper(oper), while_true(new Value(while_true)), while_false(new Value(while_false)) {}
    
        Condition(const Condition& other) {
            conditions = other.conditions;
            oper = other.oper;
            if (other.while_true) {
                while_true = new Value(*other.while_true);
            }
            if (other.while_false) {
                while_false = new Value(*other.while_false);
            }
        }
        explicit Condition(const YAML::Node& node) { parse_from(node); }

        void set_while_true(Value* value);
        void set_while_false(Value* value);

        void parse_from(const YAML::Node& node) override;
        float read(const JoyContext& context) override;
    
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

    // Define value types and parsing methods
    YAML_ENUM(ValueType,
        constant,
        axis,
        condition
    )

    // Fields
    std::variant<
        Constant,
        Axis,
        Condition
    > value{Constant(0)};

    ValueType type() const { return (ValueType)value.index(); }
    GET_VARIANT(constant, Constant, value)
    GET_VARIANT(axis, Axis, value)
    GET_VARIANT(condition, Condition, value)
    
    // Creation methods
    static Value create_constant(const Constant& _constant) { return {_constant}; }
    static Value create_constant(const float& _constant) { return create_constant(Constant(_constant)); }
    static Value create_axis(const Axis& _axis) { return {_axis}; }
    static Value create_axis(const axis_id& id = -1, const float& min = NAN, const float& max = NAN) { return create_axis(id, min, max); }
    static Value create_condition(const Condition& _condition) { return {_condition}; }
    static Value create_condition(const std::vector<Trigger>& conditions, const Condition::ConditionOperator& oper = Condition::ConditionOperator::_and, const Value& while_true = create_constant(1), const Value& while_false = create_constant(0)) 
    { return create_condition(Condition(conditions, oper, while_true, while_false)); }

    // Assignement methods
    Value& operator=(const Constant& _constant) {
        value = _constant;
        return *this;
    }
    Value& operator=(const Axis& _axis) {
        value = _axis;
        return *this;
    }
    Value& operator=(const Condition& _condition) {
        value = _condition;
        return *this;
    }

    // Constructors
    Value() = default;
    Value(const float& _constant) : value{Constant(_constant)} {}
    Value(const Constant& _constant) : value{_constant} {}
    Value(const Axis& _axis) : value{_axis} {}
    Value(const Condition& _condition) : value{_condition} {}
    explicit Value(const YAML::Node& node) { parse_from(node); }

    void parse_from(const YAML::Node& node) override;
    float read(const JoyContext& context) override;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__VALUE_HPP_