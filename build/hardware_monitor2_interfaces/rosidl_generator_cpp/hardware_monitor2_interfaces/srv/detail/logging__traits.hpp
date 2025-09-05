// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice

#ifndef HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__TRAITS_HPP_
#define HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hardware_monitor2_interfaces/srv/detail/logging__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hardware_monitor2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Logging_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_logging
  {
    out << "is_logging: ";
    rosidl_generator_traits::value_to_yaml(msg.is_logging, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Logging_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_logging
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_logging: ";
    rosidl_generator_traits::value_to_yaml(msg.is_logging, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Logging_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hardware_monitor2_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hardware_monitor2_interfaces::srv::Logging_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  hardware_monitor2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hardware_monitor2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const hardware_monitor2_interfaces::srv::Logging_Request & msg)
{
  return hardware_monitor2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hardware_monitor2_interfaces::srv::Logging_Request>()
{
  return "hardware_monitor2_interfaces::srv::Logging_Request";
}

template<>
inline const char * name<hardware_monitor2_interfaces::srv::Logging_Request>()
{
  return "hardware_monitor2_interfaces/srv/Logging_Request";
}

template<>
struct has_fixed_size<hardware_monitor2_interfaces::srv::Logging_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hardware_monitor2_interfaces::srv::Logging_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hardware_monitor2_interfaces::srv::Logging_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace hardware_monitor2_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Logging_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: logging_status
  {
    out << "logging_status: ";
    rosidl_generator_traits::value_to_yaml(msg.logging_status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Logging_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: logging_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "logging_status: ";
    rosidl_generator_traits::value_to_yaml(msg.logging_status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Logging_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hardware_monitor2_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hardware_monitor2_interfaces::srv::Logging_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  hardware_monitor2_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hardware_monitor2_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const hardware_monitor2_interfaces::srv::Logging_Response & msg)
{
  return hardware_monitor2_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<hardware_monitor2_interfaces::srv::Logging_Response>()
{
  return "hardware_monitor2_interfaces::srv::Logging_Response";
}

template<>
inline const char * name<hardware_monitor2_interfaces::srv::Logging_Response>()
{
  return "hardware_monitor2_interfaces/srv/Logging_Response";
}

template<>
struct has_fixed_size<hardware_monitor2_interfaces::srv::Logging_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hardware_monitor2_interfaces::srv::Logging_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hardware_monitor2_interfaces::srv::Logging_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hardware_monitor2_interfaces::srv::Logging>()
{
  return "hardware_monitor2_interfaces::srv::Logging";
}

template<>
inline const char * name<hardware_monitor2_interfaces::srv::Logging>()
{
  return "hardware_monitor2_interfaces/srv/Logging";
}

template<>
struct has_fixed_size<hardware_monitor2_interfaces::srv::Logging>
  : std::integral_constant<
    bool,
    has_fixed_size<hardware_monitor2_interfaces::srv::Logging_Request>::value &&
    has_fixed_size<hardware_monitor2_interfaces::srv::Logging_Response>::value
  >
{
};

template<>
struct has_bounded_size<hardware_monitor2_interfaces::srv::Logging>
  : std::integral_constant<
    bool,
    has_bounded_size<hardware_monitor2_interfaces::srv::Logging_Request>::value &&
    has_bounded_size<hardware_monitor2_interfaces::srv::Logging_Response>::value
  >
{
};

template<>
struct is_service<hardware_monitor2_interfaces::srv::Logging>
  : std::true_type
{
};

template<>
struct is_service_request<hardware_monitor2_interfaces::srv::Logging_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hardware_monitor2_interfaces::srv::Logging_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__TRAITS_HPP_
