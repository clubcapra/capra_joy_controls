#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "capra_joy_controls/yaml.hpp"
#include "capra_joy_controls/parsable.hpp"

using namespace capra_joy_controls;

struct MyStruct {
    YAML_ENUM(MyEnum1,
        ME1_1, ME1_2, ME1_3
    )
    YAML_ENUM(MyEnum2, ME2_1, ME2_2, ME2_3)
    YAML_ENUM(MyEnum3, 
        ME3_1, 
        ME3_2, 
        ME3_3
    )
};

TEST(capra_joy_controls_yaml, enum_parsing)
{
    ASSERT_EQ(MyStruct::parse_enum_MyEnum1(YAML::Node("ME1_1")), MyStruct::ME1_1);
    ASSERT_EQ(MyStruct::parse_enum_MyEnum1(YAML::Node("ME1_2")), MyStruct::ME1_2);
    ASSERT_EQ(MyStruct::parse_enum_MyEnum1(YAML::Node("ME1_3")), MyStruct::ME1_3);

    ASSERT_EQ(MyStruct::parse_enum_MyEnum2(YAML::Node("ME2_1")), MyStruct::ME2_1);
    ASSERT_EQ(MyStruct::parse_enum_MyEnum2(YAML::Node("ME2_2")), MyStruct::ME2_2);
    ASSERT_EQ(MyStruct::parse_enum_MyEnum2(YAML::Node("ME2_3")), MyStruct::ME2_3);

    ASSERT_EQ(MyStruct::parse_enum_MyEnum3(YAML::Node("ME3_1")), MyStruct::ME3_1);
    ASSERT_EQ(MyStruct::parse_enum_MyEnum3(YAML::Node("ME3_2")), MyStruct::ME3_2);
    ASSERT_EQ(MyStruct::parse_enum_MyEnum3(YAML::Node("ME3_3")), MyStruct::ME3_3);
}

TEST(capra_joy_controls_yaml, file_parsing)
{
    auto node = YAML::LoadFile(ament_index_cpp::get_package_share_directory("capra_joy_controls") + "/config/test_simple_yaml.yaml");
    expect_not_null(node);
    expect_defined(node);

    auto nparent1 = node["parent1"];
    expect_defined(nparent1);

    EXPECT_ANY_THROW(expect_defined(node["parent3"]));

    auto nsequence = node["parent1"]["sequence"];
    expect_node_type(nsequence, YAML::NodeType::Sequence);
    expect_node_type(nsequence[0], YAML::NodeType::Scalar);
    expect_node_type(nsequence[1], YAML::NodeType::Map);
}

TEST(capra_joy_controls_parsable, trigger_on_off)
{
    auto node = YAML::LoadFile(ament_index_cpp::get_package_share_directory("capra_joy_controls") + "/config/test_simple_types.yaml");

    auto off_int = Trigger(node["trigger"]["off_int"]);
    ASSERT_EQ(off_int.type(), Trigger::off);

    auto on_int = Trigger(node["trigger"]["on_int"]);
    ASSERT_EQ(on_int.type(), Trigger::on);

    ASSERT_ANY_THROW(auto trig_fail = Trigger(node["trigger"]["trig_fail"]));
}

TEST(capra_joy_controls_parsable, trigger_button)
{
    auto node = YAML::LoadFile(ament_index_cpp::get_package_share_directory("capra_joy_controls") + "/config/test_simple_types.yaml");

    auto button_short = Trigger(node["trigger"]["button_short"]);
    ASSERT_EQ(button_short.type(), Trigger::button);
    ASSERT_TRUE(button_short.get_button());
    ASSERT_EQ(button_short.get_button()->id, 2);
    
    auto button_long = Trigger(node["trigger"]["button_long"]);
    ASSERT_EQ(button_long.type(), Trigger::button);
    ASSERT_TRUE(button_long.get_button());
    ASSERT_EQ(button_long.get_button()->id, 5);

    auto button_full = Trigger(node["trigger"]["button_full"]);
    ASSERT_EQ(button_full.type(), Trigger::button);
    ASSERT_TRUE(button_full.get_button());
    ASSERT_EQ(button_full.get_button()->id, 6);
    ASSERT_EQ(button_full.get_button()->event, Trigger::Button::on_release);
}

TEST(capra_joy_controls_parsable, trigger_axis_range)
{
    auto node = YAML::LoadFile(ament_index_cpp::get_package_share_directory("capra_joy_controls") + "/config/test_simple_types.yaml");

    auto axis_range_half = Trigger(node["trigger"]["axis_range_half"]);
    ASSERT_EQ(axis_range_half.type(), Trigger::axis_range);
    ASSERT_TRUE(axis_range_half.get_axis_range());
    ASSERT_EQ(axis_range_half.get_axis_range()->id, 4);
    ASSERT_EQ(axis_range_half.get_axis_range()->min, 8.0);
    ASSERT_EQ(axis_range_half.get_axis_range()->max, 2.0);
    
    ASSERT_ANY_THROW(auto axis_range_invalid = Trigger(node["trigger"]["axis_range_invalid"]));
}

TEST(capra_joy_controls_parsable, trigger_condition)
{
    auto node = YAML::LoadFile(ament_index_cpp::get_package_share_directory("capra_joy_controls") + "/config/test_simple_types.yaml");

    auto condition_simple = Trigger(node["trigger"]["condition_simple"]);
    ASSERT_EQ(condition_simple.type(), Trigger::condition);
    ASSERT_TRUE(condition_simple.get_condition());
    ASSERT_EQ(condition_simple.get_condition()->conditions[0].type(), Trigger::on);
    
    ASSERT_ANY_THROW(auto condition_invalid = Trigger(node["trigger"]["condition_invalid"]));

    auto condition_recursive = Trigger(node["trigger"]["condition_recursive"]);
    ASSERT_EQ(condition_recursive.type(), Trigger::condition);
    ASSERT_TRUE(condition_recursive.get_condition());
    ASSERT_EQ(condition_recursive.get_condition()->while_false->type(), Trigger::condition);
    ASSERT_TRUE(condition_recursive.get_condition()->while_false->get_condition());
    ASSERT_EQ(condition_recursive.get_condition()->while_false->get_condition()->conditions[0].type(), Trigger::on);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}