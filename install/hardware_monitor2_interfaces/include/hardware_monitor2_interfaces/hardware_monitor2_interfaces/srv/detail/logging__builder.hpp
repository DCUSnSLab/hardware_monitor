// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice

#ifndef HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__BUILDER_HPP_
#define HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hardware_monitor2_interfaces/srv/detail/logging__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hardware_monitor2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Logging_Request_is_logging
{
public:
  Init_Logging_Request_is_logging()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hardware_monitor2_interfaces::srv::Logging_Request is_logging(::hardware_monitor2_interfaces::srv::Logging_Request::_is_logging_type arg)
  {
    msg_.is_logging = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hardware_monitor2_interfaces::srv::Logging_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hardware_monitor2_interfaces::srv::Logging_Request>()
{
  return hardware_monitor2_interfaces::srv::builder::Init_Logging_Request_is_logging();
}

}  // namespace hardware_monitor2_interfaces


namespace hardware_monitor2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Logging_Response_logging_status
{
public:
  Init_Logging_Response_logging_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hardware_monitor2_interfaces::srv::Logging_Response logging_status(::hardware_monitor2_interfaces::srv::Logging_Response::_logging_status_type arg)
  {
    msg_.logging_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hardware_monitor2_interfaces::srv::Logging_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::hardware_monitor2_interfaces::srv::Logging_Response>()
{
  return hardware_monitor2_interfaces::srv::builder::Init_Logging_Response_logging_status();
}

}  // namespace hardware_monitor2_interfaces

#endif  // HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__BUILDER_HPP_
