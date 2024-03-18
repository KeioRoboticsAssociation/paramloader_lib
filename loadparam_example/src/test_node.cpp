#include "paramloader_lib/paramloader_lib.hpp"
#include "rclcpp/rclcpp.hpp"

class LoadParamExample : public rclcpp::Node {
 public:
  LoadParamExample() : Node("load_param_example"), param_loader_(this) {
    bool bool_param = param_loader_.load_param("bool_param", true);
    int int_param = param_loader_.load_param("int_param", 1);
    float float_param = param_loader_.load_param("float_param", 1.1);
    std::string string_param =
        param_loader_.load_param("string_param", "hello");
    std::vector<int> int_array_param =
        param_loader_.load_param("int_array_param", std::vector<int>(0, 0));
    std::vector<double> float_array_param = param_loader_.load_param(
        "double_array_param", std::vector<double>(0, 0));
    std::vector<std::string> string_array_param = param_loader_.load_param(
        "string_array_param", std::vector<std::string>(0, ""));
    RCLCPP_INFO(get_logger(), "bool_param: %d", bool_param);
    RCLCPP_INFO(get_logger(), "int_param: %d", int_param);
    RCLCPP_INFO(get_logger(), "float_param: %f", float_param);
    RCLCPP_INFO(get_logger(), "string_param: %s", string_param.c_str());
    RCLCPP_INFO(get_logger(), "int_array_param: ");
    for (auto i : int_array_param) {
      RCLCPP_INFO(get_logger(), "%d", i);
    }
    RCLCPP_INFO(get_logger(), "float_array_param: ");
    for (auto f : float_array_param) {
      RCLCPP_INFO(get_logger(), "%f", f);
    }
    RCLCPP_INFO(get_logger(), "string_array_param: ");
    for (auto s : string_array_param) {
      RCLCPP_INFO(get_logger(), "%s", s.c_str());
    }
  }

 private:
  ParamLoaderLib param_loader_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadParamExample>());
  rclcpp::shutdown();
  return 0;
}