#pragma once
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class ParamLoaderLib {
 public:
  ParamLoaderLib(rclcpp::Node *node, bool outputflag = false)
      : node_(node), outputflag_(outputflag){};
  ~ParamLoaderLib(){};

  inline bool load_param(std::string param_name, bool default_value) {
    node_->declare_parameter(param_name, default_value);
    return node_->get_parameter(param_name).as_bool();
  }

  inline int load_param(std::string param_name, int default_value) {
    node_->declare_parameter(param_name, default_value);
    return node_->get_parameter(param_name).as_int();
  }

  inline double load_param(std::string param_name, double default_value) {
    node_->declare_parameter(param_name, default_value);
    return (double)node_->get_parameter(param_name).as_double();
  }

  inline std::string load_param(std::string param_name, char *default_value) {
    node_->declare_parameter(param_name, default_value);
    return node_->get_parameter(param_name).as_string();
  }

  inline std::vector<double> load_param(
      std::string param_name,
      std::vector<double> default_value = std::vector<double>(0, 0)) {
    node_->declare_parameter(param_name, default_value);
    return node_->get_parameter(param_name).as_double_array();
  }

  inline std::vector<int> load_param(
      std::string param_name,
      std::vector<int> default_value = std::vector<int>(0, 0)) {
    node_->declare_parameter(param_name, default_value);
    auto param = node_->get_parameter(param_name).as_integer_array();
    std::vector<int> ret;
    for (auto p : param) {
      ret.push_back((int)p);
    }
    return ret;
  }

  std::vector<std::string> load_param(std::string param_name,
                                      std::vector<std::string> default_value =
                                          std::vector<std::string>(0, "")) {
    node_->declare_parameter(param_name, default_value);
    return node_->get_parameter(param_name).as_string_array();
  }

 private:
  rclcpp::Node *node_;
  bool outputflag_;
};