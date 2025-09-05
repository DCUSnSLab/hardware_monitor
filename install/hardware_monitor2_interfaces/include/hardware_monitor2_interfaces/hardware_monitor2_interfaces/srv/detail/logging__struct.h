// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice

#ifndef HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__STRUCT_H_
#define HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'is_logging'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Logging in the package hardware_monitor2_interfaces.
typedef struct hardware_monitor2_interfaces__srv__Logging_Request
{
  rosidl_runtime_c__String is_logging;
} hardware_monitor2_interfaces__srv__Logging_Request;

// Struct for a sequence of hardware_monitor2_interfaces__srv__Logging_Request.
typedef struct hardware_monitor2_interfaces__srv__Logging_Request__Sequence
{
  hardware_monitor2_interfaces__srv__Logging_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hardware_monitor2_interfaces__srv__Logging_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'logging_status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Logging in the package hardware_monitor2_interfaces.
typedef struct hardware_monitor2_interfaces__srv__Logging_Response
{
  rosidl_runtime_c__String logging_status;
} hardware_monitor2_interfaces__srv__Logging_Response;

// Struct for a sequence of hardware_monitor2_interfaces__srv__Logging_Response.
typedef struct hardware_monitor2_interfaces__srv__Logging_Response__Sequence
{
  hardware_monitor2_interfaces__srv__Logging_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hardware_monitor2_interfaces__srv__Logging_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__STRUCT_H_
