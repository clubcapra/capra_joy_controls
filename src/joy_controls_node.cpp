#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <variant>
#include <yaml-cpp/yaml.h>
// #include <typeinfo>

#include "capra_joy_controls/params_helper.hpp"
#include "capra_joy_controls/types.hpp"
#include "capra_joy_controls/utils.hpp"
#include "capra_joy_controls/yaml.hpp"


namespace capra_joy_controls {

using namespace std::chrono_literals;

using btn_id = int;
using axis_id = int;

using Twist = geometry_msgs::msg::Twist;
using Joy = sensor_msgs::msg::Joy;
using Bool = std_msgs::msg::Bool;

template <typename TIter, typename TValue>
inline int index_of(TIter begin, TIter end, const TValue& val) {
    if (auto match = std::find(begin, end, val); match != end) {
        return (int)(match - begin);
    }
    return -1;
}

struct JoyFrame {
    // Buttons
    int A;          // Index: 0
    int B;          // Index: 1
    int X;          // Index: 2
    int Y;          // Index: 3
    int LB;         // Index: 4
    int RB;         // Index: 5
    int VIEW;       // Index: 6
    int MENU;       // Index: 7
    int XBOX;       // Index: 8  (xbox only)
    int LS_BTN;     // Index: 9
    int RS_BTN;     // Index: 10
    int SHARE;      // Index: 11 (xbox only)
    int LB4;        // Index: 12 (steamdeck only)
    int LB5;        // Index: 13 (steamdeck only)
    int RB4;        // Index: 14 (steamdeck only)
    int RB5;        // Index: 15 (steamdeck only)

    // Axes
    float LS_X;     // Index: 0     [-1;1] (left is -1)
    float LS_Y;     // Index: 1     [-1;1] (up is 1)
    float LT;       // Index: 2     [-1;1] (off is -1, full is 1)
    float RS_X;     // Index: 3     [-1;1] (left is -1)
    float RS_Y;     // Index: 4     [-1;1] (up is 1)
    float RT;       // Index: 5     [-1;1] (off is -1, full is 1)
    float D_PAD_X;  // Index: 6     [-1;1] (left is 1)
    float D_PAD_Y;  // Index: 7     [-1;1] (up is 1)
    float L_TPAD_X; // Index: 8     [-1;1] (left is -1) (steamdeck only)
    float L_TPAD_Y; // Index: 9     [-1;1] (up is 1)    (steamdeck only)
    float R_TPAD_X; // Index: 10    [-1;1] (left is -1) (steamdeck only)
    float R_TPAD_Y; // Index: 11    [-1;1] (up is 1)    (steamdeck only)
};

class YAMLParseException : public std::runtime_error {
public:
    explicit YAMLParseException(const YAML::Node& node, const std::string& what, const std::runtime_error* inner = nullptr) 
    : std::runtime_error(
        "Error parsing yaml at " + mark2str(node.Mark()) + ":" + what + (inner ? std::string("\nFrom inner:") + inner->what() : std::string())
    ) {}
private:
    std::string mark2str(const YAML::Mark& mark) {
        if (mark.is_null()) return "";
        std::ostringstream oss;
        oss << "line:" << mark.line << " col:" << mark.column << " pos:" << mark.pos;
        return oss.str();
    }
};

template <typename T>
inline std::string join(const std::vector<T>& parts, const std::string& sep) {
    std::ostringstream oss;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i > 0) oss << sep;
        oss << parts[i];
    }
    return oss.str();
}

class YAMLUndefinedException : public YAMLParseException {
public:
    explicit YAMLUndefinedException(const YAML::Node& node, const std::runtime_error* inner = nullptr)
    : YAMLParseException(node,
        "Node '" + node.Tag() + "' is undefined",
        inner
    ) {}

    explicit YAMLUndefinedException(const YAML::Node& parent, const YAML::Node& node, const std::runtime_error* inner = nullptr)
    : YAMLParseException(node,
        "Node '" + node.Tag() + "' is undefined for object '" + parent.Tag() + "'",
        inner
    ) {}
};

class YAMLNullException : public YAMLParseException {
public:
    explicit YAMLNullException(const YAML::Node& node, const std::runtime_error* inner = nullptr)
    : YAMLParseException(node,
        "Node '" + node.Tag() + "' is null",
        inner
    ) {}

    explicit YAMLNullException(const YAML::Node& parent, const YAML::Node& node, const std::runtime_error* inner = nullptr)
    : YAMLParseException(node,
        "Node '" + node.Tag() + "' is null for object '" + parent.Tag() + "'",
        inner
    ) {}
};

template <typename TEnum>
class YAMLInvalidEnumValue : public YAMLParseException {
public:
    explicit YAMLInvalidEnumValue(const YAML::Node& node, const std::string& entry, const std::vector<std::string>& allowedValues, 
        const std::runtime_error* inner = nullptr) 
    : YAMLParseException(node,
        "Invalid enum value '" + entry +
        "' for enum '" + /*typeid(TEnum).name() + */
        "'. Allowed values: " + join(allowedValues, ", "),
        inner
    ) {}
};

class YAMLInvalidNodeType : public YAMLParseException {
public:
    explicit YAMLInvalidNodeType(const YAML::Node& node, const YAML::NodeType::value& expected, const std::runtime_error* inner = nullptr) 
    : YAMLParseException(node,
        "Invalid node type '" + type2str(node.Type()) + "' expected '" + type2str(expected) + "'",
        inner
    ) {}

    explicit YAMLInvalidNodeType(const YAML::Node& node, const std::vector<YAML::NodeType::value>& expected, const std::runtime_error* inner = nullptr) 
    : YAMLParseException(node,
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
    explicit YAMLInvalidFieldType(const YAML::Node& node, const std::runtime_error* inner = nullptr) 
    : YAMLParseException(node,
        "Invalid value type for field '" + node.Tag() + "'. Expected '" + /*typeid(TField).name() + */"'",
        inner
    ) {}
};

template <typename TValue>
class YAMLInvalidValue : public YAMLParseException {
public:
    explicit YAMLInvalidValue(const YAML::Node& node, const TValue& value, const std::vector<TValue>& allowedValues, const std::runtime_error* inner = nullptr)
    : YAMLParseException(node,
        "Invalid value for field '" + node.Tag() + "' expected one of [" + join(allowedValues, ", ") + "' got '" + node.Scalar() + "'",
        inner
    ) {}
    explicit YAMLInvalidValue(const YAML::Node& node, const TValue& value, const TValue& min, const TValue& max, const std::runtime_error* inner = nullptr)
    : YAMLParseException(node,
        (std::ostringstream() << "Invalid value for field '" << node.Tag() << 
        "' expected a value between '" << min << "' and '" << max << 
        "' got '" << node.Scalar() << "'").str(),
        inner
    ) {}
};

inline void expect_not_null(const YAML::Node& node) {
    if (node.IsNull()) throw YAMLUndefinedException(node);
}

inline void expect_not_null(const YAML::Node& parent, const YAML::Node& node) {
    if (node.IsNull()) throw YAMLUndefinedException(parent, node);
}

inline void expect_defined(const YAML::Node& node) {
    if (!node.IsDefined()) throw YAMLUndefinedException(node);
}

inline void expect_defined(const YAML::Node& parent, const YAML::Node& node) {
    if (!node.IsDefined()) throw YAMLUndefinedException(parent, node);
}

inline void expect_node_type(const YAML::Node& node, const YAML::NodeType::value& type) {
    if (node.Type() != type) throw YAMLInvalidNodeType(node, type);
}

inline void expect_node_type(const YAML::Node& node, const std::vector<YAML::NodeType::value>& types) {
    if (auto match = std::find(types.cbegin(), types.cend(), node.Type()); match == types.cend()) throw YAMLInvalidNodeType(node, types);
}

template <typename TValue>
inline TValue parse_value(const YAML::Node& node) {
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Scalar);
    try {
        return node.as<TValue>();
    } catch (const std::runtime_error& e) {
        throw YAMLInvalidFieldType<TValue>(node, &e);
    }
}


inline std::vector<std::string> split(const std::string& sep, const std::string& text) {
    if (sep.empty()) return {text};

    std::vector<std::string> res;
    auto selItem = text.cbegin();

    for (auto selBegin = text.cbegin(); selBegin + sep.size() <= text.cend(); ++selBegin) {
        if (std::equal(sep.cbegin(), sep.cend(), selBegin)) {
            res.emplace_back(selItem, selBegin);
            selBegin += sep.size() - 1;  // jump to end of separator
            selItem = selBegin + 1;      // next token starts after separator
        }
    }
    // add trailing text
    res.emplace_back(selItem, text.cend());

    return res;
}


template <typename TEnum>
inline TEnum parse_any_enum(const YAML::Node& node, const std::string& value, const std::vector<std::string>& values) {
    
    for (int i = 0; i < values.size(); ++i) {
        if (value == values[i]) {
            return (TEnum)i;
        }
    }
    throw YAMLInvalidEnumValue<TEnum>(node, value, values);
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
name parse_enum_##name(const YAML::Node& node) { \
    return parse_any_enum<name>(node, split(", ", #__VA_ARGS__)); \
} \
name parse_enum_##name(const YAML::Node& node, const std::string& value) { \
    return parse_any_enum<name>(node, value, split(", ", #__VA_ARGS__)); \
}

struct YAMLParsable {
    YAMLParsable() = default;
    virtual ~YAMLParsable() = default;
    virtual void parse_from(const YAML::Node& node) = 0;
};

struct Trigger;


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
            id: 5 # Joy axis index                  | Required
            event: "inside" # Activation state      | Optional (default:inside) [options:inside, on_enter, on_exit, outside]
            min: 0.5 # Minimum value                | Optional unless max not defined (default:undefined)
            max: 1.0 # Maximum value                | Optional unless min not defined (default:undefined)
  
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
struct Trigger : YAMLParsable {
       struct Off : YAMLParsable {
           Off() = default;
           explicit Off(const YAML::Node& node) { parse_from(node); }
        void parse_from(const YAML::Node& node) override { }
    };
    struct On : YAMLParsable {
        On() = default;
        explicit On(const YAML::Node& node) { parse_from(node); }
        void parse_from(const YAML::Node& node) override { }
    };
    
    struct Button : YAMLParsable {
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

        void parse_from(const YAML::Node& node) override {
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
    };
    struct AxisRange : YAMLParsable {
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

        void parse_from(const YAML::Node& node) override {
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
            if (min == NAN && max == NAN) {
                throw YAMLParseException(node, "At least 'min' or 'max' need to be specified for axis_range");
            }
        }
    };
    struct Condition : YAMLParsable {
        std::vector<Trigger> conditions = {};
        Trigger *while_true = nullptr;
        Trigger *while_false = nullptr;

        Condition() = default;
        Condition(const std::vector<Trigger>& conditions, const Trigger& while_true = Trigger::On(), const Trigger& while_false = Trigger::Off()) 
        : conditions(conditions), while_true(new Trigger(while_true)), while_false(new Trigger(while_false)) {}
    
        Condition(const Condition& other) {
            conditions = other.conditions;
            if (other.while_true) {
                while_true = new Trigger(*other.while_true);
            }
            if (other.while_false) {
                while_false = new Trigger(*other.while_false);
            }
        }
        explicit Condition(const YAML::Node& node) { parse_from(node); }

        void set_while_true(Trigger* trigger) {
            if (while_true) {
                delete while_true;
            }
            while_true = trigger;
        }

        void set_while_false(Trigger* trigger) {
            if (while_false) {
                delete while_false;
            }
            while_false = trigger;
        }

        void parse_from(const YAML::Node& node) override {
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
                throw YAMLParseException(node, "At least one condition is required");
            }

            if (auto nwhile_true = node["while_true"]) {
                set_while_true(new Trigger(nwhile_true));
            }

            if (auto nwhile_false = node["while_false"]) {
                set_while_false(new Trigger(nwhile_false));
            }
        }
    
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
    YAML_ENUM(TriggerType, 
        off,
        on,
        button,
        axis_range,
        condition
    )
    std::variant<
        Off,
        On,
        Button,
        AxisRange,
        Condition
    > value{Off()};

    static Trigger create_off(const Off& _off = Off()) { return {_off}; }
    static Trigger create_on(const On& _on = On()) { return {_on}; }
    static Trigger create_button(const Button& button) { return {button}; }
    static Trigger create_button(const btn_id& id, const Button::ButtonEventType& event = Button::held) { return Button(id, event); }
    static Trigger create_axis_range(const AxisRange& trigger) { return {trigger}; }
    static Trigger create_axis_range(const axis_id& id, const AxisRange::AxisRangeEventType& event = AxisRange::inside, const float& min = NAN, const float& max = NAN)
    { return create_axis_range(AxisRange(id, event, min, max)); }
    static Trigger create_condition(const Condition& condition) { return {condition}; }
    static Trigger create_condition(const std::vector<Trigger>& conditions, const Trigger& while_true = On(), const Trigger& while_false = Off())
    { return create_condition(Condition{conditions, while_true, while_false}); }

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

    void parse_from(const YAML::Node& node) override {
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
            throw YAMLInvalidValue(node, v, {0, 1});
        }
        if (node.IsMap()) {
            for (auto entry : node) {
                TriggerType type = parse_enum_TriggerType(entry, entry.Tag());
                switch (type) {
                    case TriggerType::button:
                        value = Button(entry);
                        return;
                    case TriggerType::axis_range:
                        value = AxisRange(entry);
                        return;
                    case TriggerType::condition:
                        value = Condition(entry);
                        return;
                    case TriggerType::on:
                    case TriggerType::off:
                    default:
                        break;
                }
                throw YAMLInvalidValue<std::string>(entry, entry.Tag(), {"button", "axis_range", "condition"});
            }
        }
        
    }
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
        while_true: 1 # Value               | Optional (default:1)
        while_false: 0 # Value              | Optional (default:0)
```
 * @endcode
 * 
 * 
 */
struct Value : YAMLParsable {
    struct Constant : YAMLParsable {
        float value = 0;
        Constant() = default;
        Constant(const float& value) : value{value} {}
        explicit Constant(const YAML::Node& node) { parse_from(node); }
        void parse_from(const YAML::Node& node) override {
            expect_not_null(node);
            expect_defined(node);
            expect_node_type(node, YAML::NodeType::Scalar);
            value = parse_value<float>(node);
        }
    };
    struct Axis : YAMLParsable{
        axis_id id = -1;
        float min = NAN;
        float max = NAN;
        Axis() = default;
        Axis(const axis_id& id, const float& min = NAN, const float& max = NAN) 
        : id{id}, min{min}, max{max} {}
        explicit Axis(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override {
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
    };
    struct Condition : YAMLParsable {
        std::vector<Trigger> conditions = {};
        Value* while_true = nullptr;
        Value* while_false = nullptr;

        Condition() = default;
        Condition(const std::vector<Trigger>& conditions, const Value& while_true = Value::Constant(1), const Value& while_false = Value::Constant(0)) 
        : conditions(conditions), while_true(new Value(while_true)), while_false(new Value(while_false)) {}
    
        Condition(const Condition& other) {
            conditions = other.conditions;
            if (other.while_true) {
                while_true = new Value(*other.while_true);
            }
            if (other.while_false) {
                while_false = new Value(*other.while_false);
            }
        }
        explicit Condition(const YAML::Node& node) { parse_from(node); }

        void set_while_true(Value* value) {
            if (while_true) {
                delete while_true;
            }
            while_true = value;
        }

        void set_while_false(Value* value) {
            if (while_false) {
                delete while_false;
            }
            while_false = value;
        }

        void parse_from(const YAML::Node& node) override {
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
                throw YAMLParseException(node, "At least one condition is required");
            }

            if (auto nwhile_true = node["while_true"]) {
                set_while_true(new Value(nwhile_true));
            }

            if (auto nwhile_false = node["while_false"]) {
                set_while_false(new Value(nwhile_false));
            }
        }
    
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
    YAML_ENUM(ValueType,
        constant,
        axis,
        condition
    )
    std::variant<
        Constant,
        Axis,
        Condition
    > value{Constant(0)};
    
    static Value create_constant(const Constant& _constant) { return {_constant}; }
    static Value create_constant(const float& _constant) { return create_constant(Constant(_constant)); }
    static Value create_axis(const Axis& _axis) { return {_axis}; }
    static Value create_axis(const axis_id& id = -1, const float& min = NAN, const float& max = NAN) { return create_axis(id, min, max); }
    static Value create_condition(const Condition& _condition) { return {_condition}; }
    static Value create_condition(const std::vector<Trigger>& conditions, const Value& while_true = create_constant(1), const Value& while_false = create_constant(0)) 
        { return create_condition(Condition(conditions, while_true, while_false)); }

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

    Value() = default;
    Value(const float& _constant) : value{Constant(_constant)} {}
    Value(const Constant& _constant) : value{_constant} {}
    Value(const Axis& _axis) : value{_axis} {}
    Value(const Condition& _condition) : value{_condition} {}
    explicit Value(const YAML::Node& node) { parse_from(node); }

    void parse_from(const YAML::Node& node) override {
        expect_defined(node);
        expect_node_type(node, {YAML::NodeType::Map, YAML::NodeType::Scalar});

        if (node.IsScalar()) {
            /* Abridged version:
            parent:
                value: <constant>
            */
            value = Constant(parse_value<float>(node));
        }
        if (node.IsMap()) {
            for (auto entry : node) {
                ValueType type = parse_enum_ValueType(entry, entry.Tag());
                switch (type) {
                    case ValueType::constant:
                        value = Constant(entry);
                        return;
                    case ValueType::axis:
                        value = Axis(entry);
                        return;
                    case ValueType::condition:
                        value = Condition(entry);
                        return;
                    default:
                        break;
                }
                throw YAMLInvalidValue<std::string>(entry, entry.Tag(), {"button", "axis_range", "condition"});
            }
        }
    }
};


struct Action : YAMLParsable {
    struct TwistPub : YAMLParsable {
        std::string topic = "~/cmd_vel";
        Trigger enable_button = 1;
        Trigger turbo_button = 0;
        bool publish_stamped_twist = false;

        struct TeleopTwistJoy : YAMLParsable {
            struct Angular : YAMLParsable {
                Value yaw = 0, pitch = 0, roll = 0;
                Angular(const Value& yaw = 0, const Value& pitch = 0, const Value& roll = 0)
                : yaw{yaw}, pitch{pitch}, roll{roll} {}
                explicit Angular(const YAML::Node& node) { parse_from(node); }

                void parse_from(const YAML::Node& node) override {
                    expect_not_null(node);
                    expect_defined(node);
                    expect_node_type(node, YAML::NodeType::Map);

                    if (auto nyaw = node["yaw"]) {
                        yaw = Value(nyaw);
                    }
                    if (auto npitch = node["pitch"]) {
                        pitch = Value(npitch);
                    }
                    if (auto nroll = node["roll"]) {
                        roll = Value(nroll);
                    }
                }
            };
            struct Linear : YAMLParsable {
                Value x = 0, y = 0, z = 0; 
                Linear(const Value& x = 0, const Value& y = 0, const Value& z = 0)
                : x{x}, y{y}, z{z} {}
                explicit Linear(const YAML::Node& node) { parse_from(node); }

                void parse_from(const YAML::Node& node) override {
                    expect_not_null(node);
                    expect_defined(node);
                    expect_node_type(node, YAML::NodeType::Map);

                    if (auto nx = node["x"]) {
                        x = Value(nx);
                    }
                    if (auto ny = node["y"]) {
                        y = Value(ny);
                    }
                    if (auto nz = node["z"]) {
                        z = Value(nz);
                    }
                }
            };
            Angular axis_angular{0, 0, 0};
            Linear axis_linear{0, 0, 0};
            Angular scale_angular{1, 1, 1};
            Linear scale_linear{1, 1, 1};
            Angular scale_angular_turbo{1, 1, 1};
            Linear scale_linear_turbo{1, 1, 1};

            TeleopTwistJoy() {}
            TeleopTwistJoy(
                const Angular& axis_angular,
                const Linear& axis_linear,
                const Angular& scale_angular = {1, 1, 1},
                const Linear& scale_linear = {1, 1, 1},
                const Angular& scale_angular_turbo = {1, 1, 1},
                const Linear& scale_linear_turbo = {1, 1, 1}
            ) : axis_angular{axis_angular},
                axis_linear{axis_linear},
                scale_angular{scale_angular},
                scale_linear{scale_linear},
                scale_angular_turbo{scale_angular_turbo},
                scale_linear_turbo{scale_linear_turbo} 
                {}
            explicit TeleopTwistJoy(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override {
                expect_not_null(node);
                expect_defined(node);

                if (auto naxis_angular = node["axis_angular"]) {
                    axis_angular = Angular(naxis_angular);
                }
                if (auto naxis_linear = node["axis_linear"]) {
                    axis_linear = Linear(naxis_linear);
                }
                if (auto nscale_angular = node["scale_angular"]) {
                    scale_angular = Angular(nscale_angular);
                }
                if (auto nscale_linear = node["scale_linear"]) {
                    scale_linear = Linear(nscale_linear);
                }
                if (auto nscale_angular_turbo = node["scale_angular_turbo"]) {
                    scale_angular_turbo = Angular(nscale_angular_turbo);
                }
                if (auto nscale_linear_turbo = node["scale_linear_turbo"]) {
                    scale_linear_turbo = Linear(nscale_linear_turbo);
                }
            }
        };
        struct Tank : YAMLParsable {
            Value left = 0, right = 0;
            Value scale = 1;
            Value scale_turbo = 1;
            Value wheel_separation = 0;
            Value turn_multiplier = 0;
            Value wheel_radius = 0;

            Tank() = default;
            Tank(
                const Value& left,
                const Value& right,
                const Value& scale = 1,
                const Value& scale_turbo = 1,
                const Value& wheel_separation = 0,
                const Value& turn_multiplier = 0,
                const Value& wheel_radius = 0
            ) :
                left{left},
                right{right},
                scale{scale},
                scale_turbo{scale_turbo},
                wheel_separation{wheel_separation},
                turn_multiplier{turn_multiplier},
                wheel_radius{wheel_radius}
            {}

            explicit Tank(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override {
                expect_not_null(node);
                expect_defined(node);
                expect_node_type(node, YAML::NodeType::Map);

                if (auto nleft = node["left"]) {
                    left = Value(nleft);
                }
                if (auto nscale = node["scale"]) {
                    scale = Value(nscale);
                }
                if (auto nscale_turbo = node["scale_turbo"]) {
                    scale_turbo = Value(nscale_turbo);
                }
                if (auto nwheel_separation = node["wheel_separation"]) {
                    wheel_separation = Value(nwheel_separation);
                }
                if (auto nturn_multiplier = node["turn_multiplier"]) {
                    turn_multiplier = Value(nturn_multiplier);
                }
                if (auto nwheel_radius = node["wheel_radius"]) {
                    wheel_radius = Value(nwheel_radius);
                }
            }
        };
        YAML_ENUM(TwistMethod,
            teleop_twist_joy,
            tank_joy
        )
        std::variant<
            TeleopTwistJoy,
            Tank>
            value{TeleopTwistJoy()};

        TwistPub() = default;
        TwistPub(
            const TeleopTwistJoy& twist,
            const std::string& topic = "~/cmd_vel",
            const Trigger& enable_button = Trigger::create_off(),
            const Trigger& turbo_button = Trigger::create_off(),
            const bool& publish_stamped_twist = false
        ) : 
        value{twist},
        topic{topic},
        enable_button{enable_button},
        turbo_button{turbo_button},
        publish_stamped_twist{publish_stamped_twist}
        {}
        TwistPub(
            const Tank& tank,
            const std::string& topic = "~/cmd_vel",
            const Trigger& enable_button = Trigger::create_off(),
            const Trigger& turbo_button = Trigger::create_off(),
            const bool& publish_stamped_twist = false
        ) : 
        value{tank},
        topic{topic},
        enable_button{enable_button},
        turbo_button{turbo_button},
        publish_stamped_twist{publish_stamped_twist}
        {}

        explicit TwistPub(const YAML::Node& node) { parse_from(node); }

        static TwistPub create_teleop_twist_joy(
            const TeleopTwistJoy& twist,
            const std::string& topic = "~/cmd_vel",
            const Trigger& enable_button = Trigger::create_off(),
            const Trigger& turbo_button = Trigger::create_off(),
            const bool& publish_stamped_twist = false
        ) {
            return TwistPub(
                twist,
                topic,
                enable_button,
                turbo_button,
                publish_stamped_twist
            );
        }

        static TwistPub create_tank_joy(
            const Tank& tank,
            const std::string& topic = "~/cmd_vel",
            Trigger enable_button = Trigger::create_off(),
            Trigger turbo_button = Trigger::create_off(),
            bool publish_stamped_twist = false
        ) {
            return TwistPub(
                tank,
                topic,
                enable_button,
                turbo_button,
                publish_stamped_twist
            );
        }

        void parse_from(const YAML::Node& node) override {
            expect_not_null(node);
            expect_defined(node);
            expect_node_type(node, YAML::NodeType::Map);

            /* TwistTank
            parent:
                my_teleop_twist:
                    type: "twist_pub"
                    topic: <topic> (Optional)
                    enable_button: <trigger> (Optional)
                    boost_button: <trigger> (Optional)
                    publish_stamped: <bool> (Optional)

                    # EITHER
                    teleop_twist_joy: <teleop_twist_joy> (Required)
                    # OR
                    tank_joy: <tank_joy> (Required)
             */
            if (auto ntopic = node["topic"]) {
                topic = parse_value<std::string>(ntopic);
            }
            if (auto nenable_button = node["enable_button"]) {
                enable_button = Trigger(nenable_button);
            }
            if (auto nturbo_button = node["turbo_button"]) {
                turbo_button = Trigger(nturbo_button);
            }
            if (auto npublish_stamped_twist = node["publish_stamped_twist"]) {
                publish_stamped_twist = parse_value<bool>(npublish_stamped_twist);
            }

            if (auto nteleop = node["teleop_twist_joy"]) {
                value = TeleopTwistJoy(nteleop);
            } else if (auto ntank = node["tank_joy"]) {
                value = Tank(ntank);
            } else {
                throw YAMLParseException(node, "Missing twist method. Must be one of [teleop_twist_joy, tank_twist]");
            }
        }
    };

    struct FlippersPub : YAMLParsable {
        struct Flippers : YAMLParsable { 
            Value front_left = 0;
            Value rear_left = 0;
            Value front_right = 0;
            Value rear_right = 0;

            Flippers() {}
            Flippers(
                const Value& front_left,
                const Value& rear_left,
                const Value& front_right,
                const Value& rear_right
            ) : 
                front_left{front_left},
                rear_left{rear_left},
                front_right{front_right},
                rear_right{rear_right} 
            {}
            explicit Flippers(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override {
                expect_not_null(node);
                expect_defined(node);

                if (auto nfront_left = node["front_left"] ) {
                    front_left = Value(nfront_left);
                }
                if (auto nrear_left = node["rear_left"] ) {
                    rear_left = Value(nrear_left);
                }
                if (auto nfront_right = node["front_right"] ) {
                    front_right = Value(nfront_right);
                }
                if (auto nrear_right = node["rear_right"] ) {
                    rear_right = Value(nrear_right);
                }
            }
        };

        struct Preset : YAMLParsable {
            std::string name = "";
            Trigger trigger{};
            Flippers positions{};

            Preset() = default;
            Preset(const std::string& name, const Trigger& trigger = Trigger(), const Flippers& positions = Flippers())
            : name{name}, trigger{trigger}, positions{positions} {}
            explicit Preset(const YAML::Node& node) { parse_from(node); }
            void parse_from(const YAML::Node& node) override {
                expect_not_null(node);
                expect_defined(node);
                expect_node_type(node, YAML::NodeType::Map);
                name = node.Tag();

                auto ntrigger = node["trigger"];
                expect_defined(ntrigger); // Required
                trigger = Trigger(ntrigger);

                auto npositions = node["positions"];
                expect_defined(npositions); // Required
                positions = Flippers(npositions);
            }
        };
        struct Movement : YAMLParsable {
            std::string name = "";
            Flippers velocities{};

            Movement() = default;
            Movement(const std::string& name, const Flippers& velocities = {})
            : name{name}, velocities{velocities} {}
            explicit Movement(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override {
                expect_not_null(node);
                expect_defined(node);
                expect_node_type(node, YAML::NodeType::Map);

                name = node.Tag();
                auto nvelocities = node["velocities"];
                expect_defined(nvelocities); // Required
                velocities = Flippers(nvelocities);
            }
        };

        std::string topic = "~/flippers";
        std::vector<Preset> presets{};
        std::vector<Movement> movements{};

        FlippersPub() = default;
        FlippersPub(const std::string& topic, const std::vector<Preset>& presets = {}, const std::vector<Movement>& movements = {})
        : topic{topic}, presets{presets}, movements{movements} {}
        explicit FlippersPub(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override {
            expect_not_null(node);
            expect_defined(node);
            expect_node_type(node, YAML::NodeType::Map);

            if (auto ntopic = node["topic"]) {
                topic = parse_value<std::string>(ntopic);
            }

            auto npresets = node["presets"];
            expect_defined(npresets);
            expect_node_type(npresets, YAML::NodeType::Sequence);
            for (auto npreset : npresets) {
                presets.push_back(Preset(npreset));
            }
            if (presets.size() == 0) {
                throw YAMLParseException(npresets, "At least one preset is required");
            }

            auto nmovements = node["movements"];
            expect_defined(nmovements);
            expect_node_type(nmovements, YAML::NodeType::Sequence);
            for (auto nmovement : nmovements) {
                movements.push_back(Movement(nmovement));
            }
        }
    };

    struct EStopPub : YAMLParsable {
        std::string topic = "~/estop";
        Trigger trigger = 0;
        EStopPub() = default;
        EStopPub(const std::string& topic, const Trigger& trigger)
        : topic{topic}, trigger{trigger} {}
        explicit EStopPub(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override {
            expect_not_null(node);
            expect_defined(node);
            expect_node_type(node, YAML::NodeType::Map);

            if (auto ntopic = node["topic"]) { // Optional
                topic = parse_value<std::string>(ntopic);
            }

            auto ntrigger = node["trigger"];
            expect_defined(ntrigger); // Required
            trigger = Trigger(ntrigger);
        }
    };

    struct TriggerClient : YAMLParsable {
        std::string service = "~/trigger";
        Trigger trigger = 0;

        TriggerClient() = default;
        TriggerClient(const std::string& service, const Trigger& trigger)
        : service{service}, trigger{trigger} {}
        explicit TriggerClient(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override {
            expect_not_null(node);
            expect_defined(node);
            expect_node_type(node, YAML::NodeType::Map);

            if (auto nservice = node["service"]) { // Optional
                service = parse_value<std::string>(nservice);
            }

            auto ntrigger = node["trigger"];
            expect_defined(ntrigger); // Required
            trigger = Trigger(ntrigger);
        }
    };
    YAML_ENUM(ActionType,
        twist_pub,
        flippers_pub,
        estop_pub,
        trigger_client
    )
    std::variant<
        TwistPub,
        FlippersPub,
        EStopPub,
        TriggerClient
        > value{TwistPub()};

    static int index_from_type(std::string type) {
        if (type == "twist") { return 0; }
        if (type == "flippers") { return 1; }
        if (type == "estop") { return 2; }
        if (type == "trigger") { return 3; }
    }
        
    static Action create_twist(const TwistPub& _twist) { return {_twist}; }
    static Action create_flippers(const FlippersPub& _flippers) { return {_flippers}; }
    static Action create_estop(const EStopPub& _estop) { return {_estop}; }
    static Action create_trigger(const TriggerClient& _trigger) { return {_trigger}; }

    Action& operator=(const TwistPub& _twist) {
        value = _twist;
        return *this;
    }
    Action& operator=(const FlippersPub& _flippers) {
        value = _flippers;
        return *this;
    }
    Action& operator=(const EStopPub& _estop) {
        value = _estop;
        return *this;
    }
    Action& operator=(const TriggerClient& _trigger) {
        value = _trigger;
        return *this;
    }

    Action() = default;
    Action(const TwistPub& _twist) : value{_twist} {}
    Action(const FlippersPub& _flippers) : value{_flippers} {}
    Action(const EStopPub& _estop) : value{_estop} {}
    Action(const TriggerClient& _trigger) : value{_trigger} {}
    explicit Action(const YAML::Node& node) { parse_from(node); }


    void parse_from(const YAML::Node& node) override {
        expect_not_null(node);
        expect_defined(node);
        expect_node_type(node, YAML::NodeType::Map);

        auto ntype = node["type"];
        expect_defined(ntype); // Required
        ActionType type = parse_enum_ActionType(ntype);
        switch (type)
        {
        case ActionType::twist_pub:
            value = TwistPub(node);
            return;
        case ActionType::flippers_pub:
            value = FlippersPub(node);
            return;
        case ActionType::estop_pub:
            value = EStopPub(node);
            return;
        case ActionType::trigger_client:
            value = TriggerClient(node);
            return;
        default:
            throw YAMLParseException(node, "Unimplementd action '" + ntype.Scalar() + "'");
        }
    }
};

class JoyControlsNode : public rclcpp::Node
{
public:

    JoyControlsNode()
        : Node("joy_controls", "",
            rclcpp::NodeOptions().allow_undeclared_parameters(
            true).automatically_declare_parameters_from_overrides(true))
    {


        // Create subcriptions
        joySub_ = this->create_subscription<Joy>(
            "joy", 1, std::bind(&JoyControlsNode::joy_cb, this, std::placeholders::_1));

        // Create publishers
        estopPub_ = this->create_publisher<Bool>("estop", 1);
    }

    void init() {
        
    }

private:
    void parse_params() {
    }

    void joy_cb(Joy::SharedPtr joy)
    {
    }



    bool enable_;
    bool estop_;
    float spacing_;
    float turnMultiplier_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<Joy>::SharedPtr joySub_;

    rclcpp::Publisher<Bool>::SharedPtr estopPub_;

    std::vector<Action> actions_;
};
} // namespace capra_joy_controls

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<capra_joy_controls::JoyControlsNode>());
    rclcpp::shutdown();
    return 0;
}