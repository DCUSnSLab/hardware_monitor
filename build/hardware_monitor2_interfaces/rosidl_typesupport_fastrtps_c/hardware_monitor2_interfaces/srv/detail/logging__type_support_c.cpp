// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice
#include "hardware_monitor2_interfaces/srv/detail/logging__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "hardware_monitor2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hardware_monitor2_interfaces/srv/detail/logging__struct.h"
#include "hardware_monitor2_interfaces/srv/detail/logging__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // is_logging
#include "rosidl_runtime_c/string_functions.h"  // is_logging

// forward declare type support functions


using _Logging_Request__ros_msg_type = hardware_monitor2_interfaces__srv__Logging_Request;

static bool _Logging_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Logging_Request__ros_msg_type * ros_message = static_cast<const _Logging_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: is_logging
  {
    const rosidl_runtime_c__String * str = &ros_message->is_logging;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Logging_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Logging_Request__ros_msg_type * ros_message = static_cast<_Logging_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: is_logging
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->is_logging.data) {
      rosidl_runtime_c__String__init(&ros_message->is_logging);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->is_logging,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'is_logging'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hardware_monitor2_interfaces
size_t get_serialized_size_hardware_monitor2_interfaces__srv__Logging_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Logging_Request__ros_msg_type * ros_message = static_cast<const _Logging_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name is_logging
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->is_logging.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Logging_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_hardware_monitor2_interfaces__srv__Logging_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hardware_monitor2_interfaces
size_t max_serialized_size_hardware_monitor2_interfaces__srv__Logging_Request(
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

  // member: is_logging
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
    using DataType = hardware_monitor2_interfaces__srv__Logging_Request;
    is_plain =
      (
      offsetof(DataType, is_logging) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Logging_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_hardware_monitor2_interfaces__srv__Logging_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Logging_Request = {
  "hardware_monitor2_interfaces::srv",
  "Logging_Request",
  _Logging_Request__cdr_serialize,
  _Logging_Request__cdr_deserialize,
  _Logging_Request__get_serialized_size,
  _Logging_Request__max_serialized_size
};

static rosidl_message_type_support_t _Logging_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Logging_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hardware_monitor2_interfaces, srv, Logging_Request)() {
  return &_Logging_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "hardware_monitor2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "hardware_monitor2_interfaces/srv/detail/logging__struct.h"
// already included above
// #include "hardware_monitor2_interfaces/srv/detail/logging__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // logging_status
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // logging_status

// forward declare type support functions


using _Logging_Response__ros_msg_type = hardware_monitor2_interfaces__srv__Logging_Response;

static bool _Logging_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Logging_Response__ros_msg_type * ros_message = static_cast<const _Logging_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: logging_status
  {
    const rosidl_runtime_c__String * str = &ros_message->logging_status;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Logging_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Logging_Response__ros_msg_type * ros_message = static_cast<_Logging_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: logging_status
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->logging_status.data) {
      rosidl_runtime_c__String__init(&ros_message->logging_status);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->logging_status,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'logging_status'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hardware_monitor2_interfaces
size_t get_serialized_size_hardware_monitor2_interfaces__srv__Logging_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Logging_Response__ros_msg_type * ros_message = static_cast<const _Logging_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name logging_status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->logging_status.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Logging_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_hardware_monitor2_interfaces__srv__Logging_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hardware_monitor2_interfaces
size_t max_serialized_size_hardware_monitor2_interfaces__srv__Logging_Response(
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

  // member: logging_status
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
    using DataType = hardware_monitor2_interfaces__srv__Logging_Response;
    is_plain =
      (
      offsetof(DataType, logging_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Logging_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_hardware_monitor2_interfaces__srv__Logging_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Logging_Response = {
  "hardware_monitor2_interfaces::srv",
  "Logging_Response",
  _Logging_Response__cdr_serialize,
  _Logging_Response__cdr_deserialize,
  _Logging_Response__get_serialized_size,
  _Logging_Response__max_serialized_size
};

static rosidl_message_type_support_t _Logging_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Logging_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hardware_monitor2_interfaces, srv, Logging_Response)() {
  return &_Logging_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "hardware_monitor2_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hardware_monitor2_interfaces/srv/logging.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Logging__callbacks = {
  "hardware_monitor2_interfaces::srv",
  "Logging",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hardware_monitor2_interfaces, srv, Logging_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hardware_monitor2_interfaces, srv, Logging_Response)(),
};

static rosidl_service_type_support_t Logging__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Logging__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hardware_monitor2_interfaces, srv, Logging)() {
  return &Logging__handle;
}

#if defined(__cplusplus)
}
#endif
