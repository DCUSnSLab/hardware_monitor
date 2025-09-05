// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice
#include "hardware_monitor2_interfaces/srv/detail/logging__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hardware_monitor2_interfaces/srv/detail/logging__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hardware_monitor2_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
cdr_serialize(
  const hardware_monitor2_interfaces::srv::Logging_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: is_logging
  cdr << ros_message.is_logging;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hardware_monitor2_interfaces::srv::Logging_Request & ros_message)
{
  // Member: is_logging
  cdr >> ros_message.is_logging;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
get_serialized_size(
  const hardware_monitor2_interfaces::srv::Logging_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: is_logging
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.is_logging.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
max_serialized_size_Logging_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: is_logging
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hardware_monitor2_interfaces::srv::Logging_Request;
    is_plain =
      (
      offsetof(DataType, is_logging) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Logging_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hardware_monitor2_interfaces::srv::Logging_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Logging_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hardware_monitor2_interfaces::srv::Logging_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Logging_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hardware_monitor2_interfaces::srv::Logging_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Logging_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Logging_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Logging_Request__callbacks = {
  "hardware_monitor2_interfaces::srv",
  "Logging_Request",
  _Logging_Request__cdr_serialize,
  _Logging_Request__cdr_deserialize,
  _Logging_Request__get_serialized_size,
  _Logging_Request__max_serialized_size
};

static rosidl_message_type_support_t _Logging_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Logging_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hardware_monitor2_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<hardware_monitor2_interfaces::srv::Logging_Request>()
{
  return &hardware_monitor2_interfaces::srv::typesupport_fastrtps_cpp::_Logging_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hardware_monitor2_interfaces, srv, Logging_Request)() {
  return &hardware_monitor2_interfaces::srv::typesupport_fastrtps_cpp::_Logging_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hardware_monitor2_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
cdr_serialize(
  const hardware_monitor2_interfaces::srv::Logging_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: logging_status
  cdr << ros_message.logging_status;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hardware_monitor2_interfaces::srv::Logging_Response & ros_message)
{
  // Member: logging_status
  cdr >> ros_message.logging_status;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
get_serialized_size(
  const hardware_monitor2_interfaces::srv::Logging_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: logging_status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.logging_status.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hardware_monitor2_interfaces
max_serialized_size_Logging_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: logging_status
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hardware_monitor2_interfaces::srv::Logging_Response;
    is_plain =
      (
      offsetof(DataType, logging_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Logging_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hardware_monitor2_interfaces::srv::Logging_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Logging_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hardware_monitor2_interfaces::srv::Logging_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Logging_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hardware_monitor2_interfaces::srv::Logging_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Logging_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Logging_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Logging_Response__callbacks = {
  "hardware_monitor2_interfaces::srv",
  "Logging_Response",
  _Logging_Response__cdr_serialize,
  _Logging_Response__cdr_deserialize,
  _Logging_Response__get_serialized_size,
  _Logging_Response__max_serialized_size
};

static rosidl_message_type_support_t _Logging_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Logging_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hardware_monitor2_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<hardware_monitor2_interfaces::srv::Logging_Response>()
{
  return &hardware_monitor2_interfaces::srv::typesupport_fastrtps_cpp::_Logging_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hardware_monitor2_interfaces, srv, Logging_Response)() {
  return &hardware_monitor2_interfaces::srv::typesupport_fastrtps_cpp::_Logging_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace hardware_monitor2_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _Logging__callbacks = {
  "hardware_monitor2_interfaces::srv",
  "Logging",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hardware_monitor2_interfaces, srv, Logging_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hardware_monitor2_interfaces, srv, Logging_Response)(),
};

static rosidl_service_type_support_t _Logging__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Logging__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hardware_monitor2_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<hardware_monitor2_interfaces::srv::Logging>()
{
  return &hardware_monitor2_interfaces::srv::typesupport_fastrtps_cpp::_Logging__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hardware_monitor2_interfaces, srv, Logging)() {
  return &hardware_monitor2_interfaces::srv::typesupport_fastrtps_cpp::_Logging__handle;
}

#ifdef __cplusplus
}
#endif
