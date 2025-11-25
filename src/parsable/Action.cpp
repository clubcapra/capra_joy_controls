#include "capra_joy_controls/parsable/Action.hpp"

namespace capra_joy_controls::parsable {

void Action::TwistPub::TeleopTwistJoy::Angular::parse_from(const YAML::Node &node)
{
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

void Action::TwistPub::TeleopTwistJoy::Linear::parse_from(const YAML::Node &node)
{
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

void Action::TwistPub::TeleopTwistJoy::parse_from(const YAML::Node &node)
{
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

void Action::TwistPub::TeleopTwistJoy::run(const JoyContext &context)
{
    // Normal twist
    _twist.angular.x = axis_angular.pitch.read(context) * scale_angular.pitch.read(context);
    _twist.angular.y = axis_angular.roll.read(context) * scale_angular.roll.read(context);
    _twist.angular.z = axis_angular.yaw.read(context) * scale_angular.yaw.read(context);

    _twist.linear.x = axis_linear.x.read(context) * scale_linear.x.read(context);
    _twist.linear.y = axis_linear.y.read(context) * scale_linear.y.read(context);
    _twist.linear.z = axis_linear.z.read(context) * scale_linear.z.read(context);

    // Turbo twist
    _twist_turbo.angular.x = axis_angular.pitch.read(context) * scale_angular_turbo.pitch.read(context);
    _twist_turbo.angular.y = axis_angular.roll.read(context) * scale_angular_turbo.roll.read(context);
    _twist_turbo.angular.z = axis_angular.yaw.read(context) * scale_angular_turbo.yaw.read(context);

    _twist_turbo.linear.x = axis_linear.x.read(context) * scale_linear_turbo.x.read(context);
    _twist_turbo.linear.y = axis_linear.y.read(context) * scale_linear_turbo.y.read(context);
    _twist_turbo.linear.z = axis_linear.z.read(context) * scale_linear_turbo.z.read(context);
}

void Action::TwistPub::TeleopTwistJoy::update_twist(Twist& twist, Twist& twist_turbo)
{
    twist = _twist;
    twist_turbo = _twist_turbo;
}

void Action::TwistPub::Tank::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Map);

    if (auto nleft = node["left"]) {
        left = Value(nleft);
    }
    if (auto nright = node["right"]) {
        right = Value(nright);
    }
    if (auto nscale = node["scale"]) {
        scale = Value(nscale);
    }
    if (auto nscale_turbo = node["scale_turbo"]) {
        scale_turbo = Value(nscale_turbo);
    }
    if (auto nwheel_separation = node["wheel_separation"]) {
        wheel_separation = parse_value<float>(nwheel_separation);
    }
    if (auto nwheel_radius = node["wheel_radius"]) {
        wheel_radius = parse_value<float>(nwheel_radius);
    }
    if (auto nwheel_separation_multiplier = node["wheel_separation_multiplier"]) {
        wheel_separation_multiplier = parse_value<float>(nwheel_separation_multiplier);
    }
    if (auto nleft_wheel_radius_multiplier = node["left_wheel_radius_multiplier"]) {
        left_wheel_radius_multiplier = parse_value<float>(nleft_wheel_radius_multiplier);
    }
    if (auto nright_wheel_radius_multiplier = node["right_wheel_radius_multiplier"]) {
        right_wheel_radius_multiplier = parse_value<float>(nright_wheel_radius_multiplier);
    }
}

void Action::TwistPub::Tank::update_twist(Twist& twist, Twist& twist_turbo)
{
    twist = _twist;
    twist_turbo = _twist_turbo;
}

void Action::TwistPub::Tank::run(const JoyContext &context)
{
    // Read values
    float left_axis = left.read(context);
    float right_axis = right.read(context);
    float normal_scale = scale.read(context);
    float turbo_scale = scale_turbo.read(context);

    // Apply multipliers
    float ws = wheel_separation * wheel_separation_multiplier;
    float lwr = wheel_radius * left_wheel_radius_multiplier;
    float rwr = wheel_radius * right_wheel_radius_multiplier;

    // Invert diff_drive equations
    float lin = (left_axis * lwr + right_axis * rwr) / 2.0;
    float ang = (right_axis * rwr - left_axis * lwr) / ws;


    _twist.linear.x = lin * normal_scale;
    _twist.angular.z = ang * normal_scale;

    _twist_turbo.linear.x = lin * turbo_scale;
    _twist_turbo.angular.z = ang * turbo_scale;
}

void Action::TwistPub::parse_from(const YAML::Node &node)
{
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
    if (auto nrate = node["rate"]) {
        rate = parse_value<float>(nrate);
    }

    if (auto nteleop = node["teleop_twist_joy"]) {
        value = TeleopTwistJoy(nteleop);
    } else if (auto ntank = node["tank_joy"]) {
        value = Tank(ntank);
    } else {
        THROW(YAMLParseException, node, "Missing twist method. Must be one of [teleop_twist_joy, tank_twist]");
    }
}

void Action::TwistPub::init(rclcpp::Node::SharedPtr node)
{
    _twist_pub = node->create_publisher<Twist>(topic, rclcpp::SystemDefaultsQoS());

    auto period = std::chrono::nanoseconds(
        static_cast<uint64_t>(1e9 / rate)
    );

    _timer = node->create_wall_timer(
        period,
        std::bind(&Action::TwistPub::_task, this)
    );
}

void Action::TwistPub::run(const JoyContext &context)
{
    // Run actions
    Twist normal_twist, turbo_twist;
    switch (value.index()) {
        case TwistMethod::teleop_twist_joy:
        {
            auto twist_joy = std::get<TeleopTwistJoy>(value);
            twist_joy.run(context);
            twist_joy.update_twist(normal_twist, turbo_twist);
        }
        break;
        case TwistMethod::tank_joy:
        {
            auto tank_joy = std::get<Tank>(value);
            tank_joy.run(context);
            tank_joy.update_twist(normal_twist, turbo_twist);
        }
        break;
        default:
        break;
    }

    bool turbo = turbo_button.read(context);
    // Get twist
    if (!enable_button.read(context)) {
        // Send zero
        _twist = Twist();
    } else {
        if (!turbo) {
            _twist = normal_twist;
        } else {
            _twist = turbo_twist;
        }
    }
}

void Action::TwistPub::_task()
{
    _twist_pub->publish(_twist);
}

void Action::FlippersPub::FlippersValues::parse_from(const YAML::Node &node)
{
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

void Action::FlippersPub::Preset::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Map);
    name = node.Tag();

    auto ntrigger = node["trigger"];
    expect_defined(ntrigger); // Required
    trigger = Trigger(ntrigger);

    auto npositions = node["positions"];
    expect_defined(npositions); // Required
    positions = FlippersValues(npositions);
}

void Action::FlippersPub::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Map);

    auto nenable = node["enable"];
    expect_defined(nenable); // Required
    enable = Trigger(nenable);

    if (auto ntopic = node["topic"]) {
        topic = parse_value<std::string>(ntopic);
    }

    if (auto npresets = node["presets"]) {
        expect_node_type(npresets, YAML::NodeType::Map);
        for (auto npreset : npresets) {
            auto p = npreset.second;
            p.SetTag(npreset.first.Scalar());
            presets.push_back(Preset(p));
        }
        if (presets.size() == 0) {
            THROW(YAMLParseException, npresets, "At least one preset is required");
        }
    }

    if (auto nmovements = node["movements"]){
        movements = FlippersValues(nmovements);
    }
    
}

void Action::FlippersPub::init(rclcpp::Node::SharedPtr node)
{
    _flippers_pub = node->create_publisher<Flippers>(topic, rclcpp::SystemDefaultsQoS());

    auto period = std::chrono::nanoseconds(
        static_cast<uint64_t>(1e9 / rate)
    );

    _timer = node->create_wall_timer(
        period,
        std::bind(&Action::FlippersPub::_task, this)
    );
}

void Action::FlippersPub::run(const JoyContext &context)
{
    // Read values
    float fl = movements.front_left.read(context);
    float rl = movements.rear_left.read(context);
    float fr = movements.front_right.read(context);
    float rr = movements.rear_right.read(context);
    bool en = enable.read(context);

    for (auto p : presets) {
        bool trig = p.trigger.read(context);
        float pfl = p.positions.front_left.read(context);
        float prl = p.positions.rear_left.read(context);
        float pfr = p.positions.front_right.read(context);
        float prr = p.positions.rear_right.read(context);

        if (trig) {
            // A position control command was sent
            _set_positions(pfl, prl, pfr, prr);
        }
    }

    auto is_small = [](const float& v){ return std::abs(v) < 0.01f; };

    if (!is_small(fl) || !is_small(rl) || !is_small(fr) || !is_small(rr)) {
        // Velocity control takes precedance
        _set_velocities(fl, rl, fr, rr);
    }
}

void Action::FlippersPub::_task()
{
    _flippers_pub->publish(_flippers);
}

void Action::FlippersPub::_set_positions(const float &front_left, const float &rear_left, const float &front_right, const float &rear_right)
{
    _flippers.control_mode = Flippers::POSITION_CONTROL_MODE;
    _flippers.front_left = front_left;
    _flippers.rear_left = rear_left;
    _flippers.front_right = front_right;
    _flippers.rear_right = rear_right;
}

void Action::FlippersPub::_set_velocities(const float &front_left, const float &rear_left, const float &front_right, const float &rear_right)
{
    _flippers.control_mode = Flippers::VELOCITY_CONTROL_MODE;
    _flippers.front_left = front_left;
    _flippers.rear_left = rear_left;
    _flippers.front_right = front_right;
    _flippers.rear_right = rear_right;
}

void Action::EStopPub::parse_from(const YAML::Node &node)
{
    expect_not_null(node);
    expect_defined(node);
    expect_node_type(node, YAML::NodeType::Map);

    if (auto ntopic = node["topic"]) { // Optional
        topic = parse_value<std::string>(ntopic);
    }

    auto nlatch = node["latch"];
    expect_defined(nlatch); // Required
    latch = Trigger(nlatch);

    auto nunlatch = node["unlatch"];
    expect_defined(nunlatch); // Required
    unlatch = Trigger(nunlatch);
}

void Action::EStopPub::init(rclcpp::Node::SharedPtr node)
{
    _estop_pub = node->create_publisher<Bool>(topic, rclcpp::SystemDefaultsQoS());
}

void Action::EStopPub::run(const JoyContext &context)
{
    bool l = latch.read(context);
    bool u = unlatch.read(context);

    if (l) {
        // Latching has precedence
        Bool b;
        b.data = true;
        _estop_pub->publish(b);
    } else {
        Bool b;
        b.data = false;
        _estop_pub->publish(b);
    }
}

void Action::TriggerClient::parse_from(const YAML::Node & node)
{
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

void Action::TriggerClient::init(rclcpp::Node::SharedPtr node)
{
    _client = node->create_client<TriggerSrv>(service);
}

void Action::TriggerClient::run(const JoyContext &context)
{
    // TODO Implement this
}

void Action::parse_from(const YAML::Node &node)
{
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
        THROW(YAMLParseException, node, "Unimplemented action '" + ntype.Scalar() + "'");
    }
}

void Action::init(rclcpp::Node::SharedPtr node)
{
    switch (value.index())
    {
    case ActionType::twist_pub:
        std::get<TwistPub>(value).init(node);
        return;
    case ActionType::flippers_pub:
        std::get<FlippersPub>(value).init(node);
        return;
    case ActionType::estop_pub:
        std::get<EStopPub>(value).init(node);
        return;
    case ActionType::trigger_client:
        std::get<TriggerClient>(value).init(node);
        return;
    default:
        break;
    }
}

void Action::run(const JoyContext &context)
{
    switch (value.index())
    {
    case ActionType::twist_pub:
        std::get<TwistPub>(value).run(context);
        return;
    case ActionType::flippers_pub:
        std::get<FlippersPub>(value).run(context);
        return;
    case ActionType::estop_pub:
        std::get<EStopPub>(value).run(context);
        return;
    case ActionType::trigger_client:
        std::get<TriggerClient>(value).run(context);
        return;
    default:
        break;
    }
}
}
