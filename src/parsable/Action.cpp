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

void Action::TwistPub::Tank::parse_from(const YAML::Node &node)
{
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

    if (auto nteleop = node["teleop_twist_joy"]) {
        value = TeleopTwistJoy(nteleop);
    } else if (auto ntank = node["tank_joy"]) {
        value = Tank(ntank);
    } else {
        THROW(YAMLParseException, node, "Missing twist method. Must be one of [teleop_twist_joy, tank_twist]");
    }
}

void Action::FlippersPub::Flippers::parse_from(const YAML::Node &node)
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
    positions = Flippers(npositions);
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
        movements = Flippers(nmovements);
    }
    
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
        THROW(YAMLParseException, node, "Unimplementd action '" + ntype.Scalar() + "'");
    }
}

}
