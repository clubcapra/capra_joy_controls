#ifndef CAPRA_JOY_CONTROLS__PARSABLE__ACTION_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__ACTION_HPP_

#include "capra_joy_controls/parsable/Trigger.hpp"
#include "capra_joy_controls/parsable/Value.hpp"

namespace capra_joy_controls::parsable {

struct Action : YAMLParsable {
    // Twist publisher
    struct TwistPub : YAMLParsable {
        // teleop_twist_joy-style twist publisher
        struct TeleopTwistJoy : YAMLParsable {
            // A yaw pitch roll container
            struct Angular : YAMLParsable {
                Value yaw = 0, pitch = 0, roll = 0;

                Angular(const Value& yaw = 0, const Value& pitch = 0, const Value& roll = 0)
                : yaw{yaw}, pitch{pitch}, roll{roll} {}
                explicit Angular(const YAML::Node& node) { parse_from(node); }

                void parse_from(const YAML::Node& node) override;
            };
            struct Linear : YAMLParsable {
                Value x = 0, y = 0, z = 0; 
                Linear(const Value& x = 0, const Value& y = 0, const Value& z = 0)
                : x{x}, y{y}, z{z} {}
                explicit Linear(const YAML::Node& node) { parse_from(node); }

                void parse_from(const YAML::Node& node) override;
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

            void parse_from(const YAML::Node& node);
        };
        // Tank-style twist publisher
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

            void parse_from(const YAML::Node& node) override;
        };

        // Define twist types and parsing methods
        YAML_ENUM(TwistMethod,
            teleop_twist_joy,
            tank_joy
        )

        // Fields
        std::string topic = "~/cmd_vel";
        Trigger enable_button = 1;
        Trigger turbo_button = 0;
        bool publish_stamped_twist = false;
        std::variant<
            TeleopTwistJoy,
            Tank
        > value{TeleopTwistJoy()};

        // Contructors
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

        // Creation methods
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

        void parse_from(const YAML::Node& node) override;
    };

    // FLippers publisher
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

            void parse_from(const YAML::Node& node) override;
        };

        struct Preset : YAMLParsable {
            std::string name = "";
            Trigger trigger{};
            Flippers positions{};

            Preset() = default;
            Preset(const std::string& name, const Trigger& trigger = Trigger(), const Flippers& positions = Flippers())
            : name{name}, trigger{trigger}, positions{positions} {}
            explicit Preset(const YAML::Node& node) { parse_from(node); }
            void parse_from(const YAML::Node& node) override;
        };
        struct Movement : YAMLParsable {
            std::string name = "";
            Flippers velocities{};

            Movement() = default;
            Movement(const std::string& name, const Flippers& velocities = {})
            : name{name}, velocities{velocities} {}
            explicit Movement(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override;
        };

        // Fields
        std::string topic = "~/flippers";
        std::vector<Preset> presets{};
        std::vector<Movement> movements{};

        FlippersPub() = default;
        FlippersPub(const std::string& topic, const std::vector<Preset>& presets = {}, const std::vector<Movement>& movements = {})
        : topic{topic}, presets{presets}, movements{movements} {}
        explicit FlippersPub(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
    };

    // EStop publisher
    struct EStopPub : YAMLParsable {
        std::string topic = "~/estop";
        Trigger trigger = 0;

        EStopPub() = default;
        EStopPub(const std::string& topic, const Trigger& trigger)
        : topic{topic}, trigger{trigger} {}
        explicit EStopPub(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
    };

    struct TriggerClient : YAMLParsable {
        std::string service = "~/trigger";
        Trigger trigger = 0;

        TriggerClient() = default;
        TriggerClient(const std::string& service, const Trigger& trigger)
        : service{service}, trigger{trigger} {}
        explicit TriggerClient(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
    };

    // Define action types and parsing methods
    YAML_ENUM(ActionType,
        twist_pub,
        flippers_pub,
        estop_pub,
        trigger_client
    )

    // Fields
    std::variant<
        TwistPub,
        FlippersPub,
        EStopPub,
        TriggerClient
    > value{TwistPub()};

    ActionType type() const { return (ActionType)value.index(); }
    GET_VARIANT(twist_pub, TwistPub, value)
    GET_VARIANT(flippers_pub, FlippersPub, value)
    GET_VARIANT(estop_pub, EStopPub, value)
    GET_VARIANT(trigger_client, TriggerClient, value)

    // Creation methods
    static Action create_twist(const TwistPub& _twist) { return {_twist}; }
    static Action create_flippers(const FlippersPub& _flippers) { return {_flippers}; }
    static Action create_estop(const EStopPub& _estop) { return {_estop}; }
    static Action create_trigger(const TriggerClient& _trigger) { return {_trigger}; }

    // Assignement operators
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

    // Contructors
    Action() = default;
    Action(const TwistPub& _twist) : value{_twist} {}
    Action(const FlippersPub& _flippers) : value{_flippers} {}
    Action(const EStopPub& _estop) : value{_estop} {}
    Action(const TriggerClient& _trigger) : value{_trigger} {}
    explicit Action(const YAML::Node& node) { parse_from(node); }

    void parse_from(const YAML::Node& node) override;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__ACTION_HPP_