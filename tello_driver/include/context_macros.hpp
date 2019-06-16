#ifndef CONTEXT_MACROS_HPP
#define CONTEXT_MACROS_HPP

// Define CXT_MACRO_MEMBER to CXT_MACRO_DEFINE_MEMBER before declaring members
#define CXT_MACRO_DEFINE_MEMBER(n, t, d) t n##_{d};


// Define CXT_MACRO_MEMBER to CXT_MACRO_LOAD_PARAMETER before invoking CXT_MACRO_INIT_PARAMETERS
#define CXT_MACRO_LOAD_PARAMETER(node_ref, cxt_ref, n, t, d) \
  cxt_ref.n##_ = node_ref.declare_parameter(#n, cxt_ref.n##_);

// Initialize the parameter members from the node
#define CXT_MACRO_INIT_PARAMETERS(all_params, validate_func) \
all_params \
validate_func(); \


// Define CXT_MACRO_MEMBER to CXT_MACRO_PARAMETER_CHANGED before invoking CXT_MACRO_REGISTER_PARAMETERS_CHANGED
// Notice that the_logger and param_set are expected to be defined and initialized
// in the CXT_MACRO_REGISTER_PARAMETERS_CHANGED macro
#define CXT_MACRO_PARAMETER_CHANGED(cxt_ref, n, t) \
if (parameter.get_name() == #n) {\
  param_set = true; \
  cxt_ref.n##_ = parameter.get_value<t>(); \
  RCLCPP_INFO(the_logger, "Parameter %s changed value to %s", #n, \
  rclcpp::to_string(rclcpp::ParameterValue{cxt_ref.n##_}).c_str()); \
}

// Register for parameter changed notifications
#define CXT_MACRO_REGISTER_PARAMETERS_CHANGED(node_ref, all_params, validate_func) \
node_ref.set_on_parameters_set_callback( \
[this, existing_callback = node_ref.set_on_parameters_set_callback(nullptr), the_logger = node_ref.get_logger()]\
(std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult\
{\
  auto result = rcl_interfaces::msg::SetParametersResult(); \
  if (nullptr != existing_callback) { \
    result = existing_callback(parameters); \
    if (!result.successful) { \
      return result; \
    } \
  } \
  bool param_set{false}; \
  for (const auto &parameter : parameters) { \
    all_params \
  } \
  if (param_set) { validate_func(); }\
  result.successful = true; \
  return result; \
});


// Define CXT_MACRO_MEMBER to CXT_MACRO_LOG_PARAMETER before logging the current value of the parameters
#define CXT_MACRO_LOG_PARAMETER(rcl_macro, logger, cxt_ref, n, t, d) \
  rcl_macro(logger, "%s = %s", #n, \
  rclcpp::to_string(rclcpp::ParameterValue{cxt_ref.n##_}).c_str());

#endif // CONTEXT_MACROS_HPP
