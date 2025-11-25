#ifndef CAPRA_JOY_CONTROLS__PARSABLE__ACTION_HPP_
#define CAPRA_JOY_CONTROLS__PARSABLE__ACTION_HPP_

#include "capra_joy_controls/parsable/Trigger.hpp"
#include "capra_joy_controls/parsable/Value.hpp"

namespace capra_joy_controls::parsable {

struct RunnableAction {
    virtual void init(rclcpp::Node::SharedPtr node) {}
    virtual void run(const JoyContext& context) = 0;
};

struct TwistAction {
    virtual void update_twist(Twist& twist, Twist& twist_turbo) = 0;
};

struct Action : YAMLParsable, RunnableAction {
    // Twist publisher
    struct TwistPub : YAMLParsable, RunnableAction {
        // teleop_twist_joy-style twist publisher
        struct TeleopTwistJoy : YAMLParsable, RunnableAction, TwistAction {
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
            // Parameters
            Angular axis_angular{0, 0, 0};
            Linear axis_linear{0, 0, 0};
            Angular scale_angular{1, 1, 1};
            Linear scale_linear{1, 1, 1};
            Angular scale_angular_turbo{1, 1, 1};
            Linear scale_linear_turbo{1, 1, 1};

            Twist _twist;
            Twist _twist_turbo;

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

            void parse_from(const YAML::Node& node) override;
            void run(const JoyContext& context) override;
            void update_twist(Twist& twist, Twist& twist_turbo) override;
        };
        // Tank-style twist publisher
        struct Tank : YAMLParsable, RunnableAction, TwistAction {
            // Parameters
            Value left = 0, right = 0;
            Value scale = 1;
            Value scale_turbo = 1;
            float wheel_separation = 1;
            float wheel_radius = 1;
            float wheel_separation_multiplier = 1;
            float left_wheel_radius_multiplier = -1;
            float right_wheel_radius_multiplier = 1;

            Twist _twist;
            Twist _twist_turbo;

            Tank() = default;
            Tank(
                const Value& left,
                const Value& right,
                const Value& scale = 1,
                const Value& scale_turbo = 1,
                const float& wheel_separation = 1,
                const float& wheel_radius = 1,
                const float& wheel_separation_multiplier = 1,
                const float& left_wheel_radius_multiplier = -1,
                const float& right_wheel_radius_multiplier = 1
            ) :
                left{left},
                right{right},
                scale{scale},
                scale_turbo{scale_turbo},
                wheel_separation{wheel_separation},
                wheel_radius{wheel_radius},
                wheel_separation_multiplier{wheel_separation_multiplier},
                left_wheel_radius_multiplier{left_wheel_radius_multiplier},
                right_wheel_radius_multiplier{right_wheel_radius_multiplier}
            {}

            explicit Tank(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override;
            void update_twist(Twist& twist, Twist& twist_turbo) override;
            void run(const JoyContext& context) override;
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
        float rate = 20;
        bool publish_stamped_twist = false;
        std::variant<
            TeleopTwistJoy,
            Tank
        > value{TeleopTwistJoy()};

        TwistMethod type() const { return (TwistMethod)value.index(); }
        GET_VARIANT(teleop_twist_joy, TeleopTwistJoy, value)
        GET_VARIANT(tank_joy, Tank, value)

        // Members
        rclcpp::Publisher<Twist>::SharedPtr _twist_pub;
        Twist _twist;
        rclcpp::TimerBase::SharedPtr _timer;

        // Contructors
        TwistPub() = default;
        TwistPub(
            const TeleopTwistJoy& twist,
            const std::string& topic = "~/cmd_vel",
            const Trigger& enable_button = Trigger::create_off(),
            const Trigger& turbo_button = Trigger::create_off(),
            const bool& publish_stamped_twist = false,
            const float& rate = 20
        ) : 
        value{twist},
        topic{topic},
        enable_button{enable_button},
        turbo_button{turbo_button},
        publish_stamped_twist{publish_stamped_twist},
        rate{rate}
        {}
        TwistPub(
            const Tank& tank,
            const std::string& topic = "~/cmd_vel",
            const Trigger& enable_button = Trigger::create_off(),
            const Trigger& turbo_button = Trigger::create_off(),
            const bool& publish_stamped_twist = false,
            const float& rate = 20
        ) : 
        value{tank},
        topic{topic},
        enable_button{enable_button},
        turbo_button{turbo_button},
        publish_stamped_twist{publish_stamped_twist},
        rate{rate}
        {}
        explicit TwistPub(const YAML::Node& node) { parse_from(node); }

        // Creation methods
        static TwistPub create_teleop_twist_joy(
            const TeleopTwistJoy& twist,
            const std::string& topic = "~/cmd_vel",
            const Trigger& enable_button = Trigger::create_off(),
            const Trigger& turbo_button = Trigger::create_off(),
            const bool& publish_stamped_twist = false,
            const float& rate = 20
        ) {
            return TwistPub(
                twist,
                topic,
                enable_button,
                turbo_button,
                publish_stamped_twist,
                rate
            );
        }
        static TwistPub create_tank_joy(
            const Tank& tank,
            const std::string& topic = "~/cmd_vel",
            Trigger enable_button = Trigger::create_off(),
            Trigger turbo_button = Trigger::create_off(),
            bool publish_stamped_twist = false,
            const float& rate = 20
        ) {
            return TwistPub(
                tank,
                topic,
                enable_button,
                turbo_button,
                publish_stamped_twist,
                rate
            );
        }

        void parse_from(const YAML::Node& node) override;
        void init(rclcpp::Node::SharedPtr node) override;
        void run(const JoyContext& context) override;

        void _task();
    };

    // FLippers publisher
    struct FlippersPub : YAMLParsable, RunnableAction {
        struct FlippersValues : YAMLParsable { 
            // Parameters
            Value front_left = 0;
            Value rear_left = 0;
            Value front_right = 0;
            Value rear_right = 0;

            FlippersValues() {}
            FlippersValues(
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
            explicit FlippersValues(const YAML::Node& node) { parse_from(node); }

            void parse_from(const YAML::Node& node) override;
        };

        struct Preset : YAMLParsable {
            std::string name = "";
            Trigger trigger{};
            FlippersValues positions{};

            Preset() = default;
            Preset(const std::string& name, const Trigger& trigger = Trigger(), const FlippersValues& positions = FlippersValues())
            : name{name}, trigger{trigger}, positions{positions} {}
            explicit Preset(const YAML::Node& node) { parse_from(node); }
            void parse_from(const YAML::Node& node) override;
        };

        // Fields
        std::string topic = "~/flippers";
        Trigger enable{};
        std::vector<Preset> presets{};
        FlippersValues movements{};
        float rate = 20;

        // Members
        rclcpp::Publisher<Flippers>::SharedPtr _flippers_pub;
        Flippers _flippers;
        rclcpp::TimerBase::SharedPtr _timer;

        FlippersPub() = default;
        FlippersPub(
            const std::string& topic,
            const Trigger& enable = {},
            const std::vector<Preset>& presets = {},
            const FlippersValues& movements = {},
            const float& rate = 20
        )
        : topic{topic}, presets{presets}, movements{movements}, rate{rate} {}
        explicit FlippersPub(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
        void init(rclcpp::Node::SharedPtr node) override;
        void run(const JoyContext& context) override;
        void _task();
        void _set_positions(
            const float& front_left,
            const float& rear_left,
            const float& front_right,
            const float& rear_right
        );
        void _set_velocities(
            const float& front_left,
            const float& rear_left,
            const float& front_right,
            const float& rear_right
        );
    };

    // EStop publisher
    struct EStopPub : YAMLParsable, RunnableAction {
        // Parameters
        std::string topic = "~/estop";
        Trigger latch = 0;
        Trigger unlatch = 0;

        // Members
        rclcpp::Publisher<Bool>::SharedPtr _estop_pub;

        EStopPub() = default;
        EStopPub(const std::string& topic, const Trigger& latch, const Trigger& unlatch)
        : topic{topic}, latch{latch}, unlatch{unlatch} {}
        explicit EStopPub(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
        void init(rclcpp::Node::SharedPtr node) override;
        void run(const JoyContext& context) override;
    };

    struct TriggerClient : YAMLParsable, RunnableAction {
        // Parameters
        std::string service = "~/trigger";
        Trigger trigger = 0;

        // Members
        rclcpp::Client<TriggerSrv>::SharedPtr _client;
        bool _lastValue;


        TriggerClient() = default;
        TriggerClient(const std::string& service, const Trigger& trigger)
        : service{service}, trigger{trigger} {}
        explicit TriggerClient(const YAML::Node& node) { parse_from(node); }

        void parse_from(const YAML::Node& node) override;
        void init(rclcpp::Node::SharedPtr node) override;
        void run(const JoyContext& context) override;
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
    void init(rclcpp::Node::SharedPtr node) override;
    void run(const JoyContext& context) override;
};

} // capra_joy_controls::parsable

#endif // CAPRA_JOY_CONTROLS__PARSABLE__ACTION_HPP_