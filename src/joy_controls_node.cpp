#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <typeinfo>

#include "capra_joy_controls/types.hpp"
#include "capra_joy_controls/utils.hpp"
#include "capra_joy_controls/yaml.hpp"
#include "capra_joy_controls/parsable.hpp"


namespace capra_joy_controls {

using namespace std::chrono_literals;

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
    // Converts the yaml parameters from ros into a more convienient YAML::Node to allow easy parsing
    YAML::Node get_yaml()
    {
        YAML::Node root;

        // Get all parameter names
        auto parameters = this->list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);

        for (const auto & name : parameters.names) {
            // Retrieve the parameter
            rclcpp::Parameter param = this->get_parameter(name);

            // Build nested YAML structure for dot-separated names (e.g. "axes.linear")
            YAML::Node *current = &root;
            std::stringstream ss(name);
            std::string token;

            while (std::getline(ss, token, '.')) {
                // current = &((*current)[token]);
            }

            // Assign value according to type
            switch (param.get_type()) {
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    *current = param.as_bool();
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    *current = param.as_int();
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    *current = param.as_double();
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING:
                    *current = param.as_string();
                    break;
                case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
                    *current = param.as_bool_array();
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
                    *current = param.as_integer_array();
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
                    *current = param.as_double_array();
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
                    *current = param.as_string_array();
                    break;
                default:
                    RCLCPP_WARN(this->get_logger(), "Unsupported parameter type for '%s'", name.c_str());
                    break;
            }
        }

        return root;
    }

    void parse_params() {
        auto params = get_yaml();
        map_.parse_from(params);
    }

    void joy_cb(Joy::SharedPtr joy)
    {
    }



    bool estop_;
    float spacing_;
    float turnMultiplier_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<Joy>::SharedPtr joySub_;

    rclcpp::Publisher<Bool>::SharedPtr estopPub_;

    std::vector<Action> actions_;
    
    // Parameters
    SchemeMap map_;
};
} // namespace capra_joy_controls

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<capra_joy_controls::JoyControlsNode>());
    rclcpp::shutdown();
    return 0;
}