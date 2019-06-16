#ifndef CONTEXT_MACROS_HPP
#define CONTEXT_MACROS_HPP

#define CXT_MACRO_DEFINE_MEMBER(n, t, d) t n##_{d};

#define CXT_MACRO_LOAD_PARAMETER(node_ref, cxt_ref, n, t, d) \
  node_ref.declare_parameter(#n); \
  node_ref.set_parameter(rclcpp::Parameter(#n, cxt_ref.n##_));

// TODO catch type exceptions
// Notice that the_logger is expected to be defined in the CXT_MACRO_REGISTER_PARAMETERS_CHANGED macro
#define CXT_MACRO_PARAMETER_CHANGED(cxt_ref, n, t) \
if (parameter.get_name() == #n) {\
  cxt_ref.n##_ = parameter.get_value<t>(); \
  RCLCPP_INFO(the_logger, "Parameter %s changed value", #n); \
}

// Initialize the context struct
#define CXT_MACRO_INIT_PARAMETERS(all_params, validate_func) \
all_params \
validate_func() \

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
  for (const auto &parameter : parameters) { \
    all_params \
  } \
  validate_func(); \
  result.successful = true; \
  return result; \
});

#endif // CONTEXT_MACROS_HPP
