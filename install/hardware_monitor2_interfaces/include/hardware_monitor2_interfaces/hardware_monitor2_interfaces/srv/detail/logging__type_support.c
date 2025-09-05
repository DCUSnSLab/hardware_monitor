// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hardware_monitor2_interfaces/srv/detail/logging__rosidl_typesupport_introspection_c.h"
#include "hardware_monitor2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hardware_monitor2_interfaces/srv/detail/logging__functions.h"
#include "hardware_monitor2_interfaces/srv/detail/logging__struct.h"


// Include directives for member types
// Member `is_logging`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hardware_monitor2_interfaces__srv__Logging_Request__init(message_memory);
}

void hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_fini_function(void * message_memory)
{
  hardware_monitor2_interfaces__srv__Logging_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_member_array[1] = {
  {
    "is_logging",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hardware_monitor2_interfaces__srv__Logging_Request, is_logging),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_members = {
  "hardware_monitor2_interfaces__srv",  // message namespace
  "Logging_Request",  // message name
  1,  // number of fields
  sizeof(hardware_monitor2_interfaces__srv__Logging_Request),
  hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_member_array,  // message members
  hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_type_support_handle = {
  0,
  &hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hardware_monitor2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging_Request)() {
  if (!hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_type_support_handle.typesupport_identifier) {
    hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hardware_monitor2_interfaces__srv__Logging_Request__rosidl_typesupport_introspection_c__Logging_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hardware_monitor2_interfaces/srv/detail/logging__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hardware_monitor2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hardware_monitor2_interfaces/srv/detail/logging__functions.h"
// already included above
// #include "hardware_monitor2_interfaces/srv/detail/logging__struct.h"


// Include directives for member types
// Member `logging_status`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hardware_monitor2_interfaces__srv__Logging_Response__init(message_memory);
}

void hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_fini_function(void * message_memory)
{
  hardware_monitor2_interfaces__srv__Logging_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_member_array[1] = {
  {
    "logging_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hardware_monitor2_interfaces__srv__Logging_Response, logging_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_members = {
  "hardware_monitor2_interfaces__srv",  // message namespace
  "Logging_Response",  // message name
  1,  // number of fields
  sizeof(hardware_monitor2_interfaces__srv__Logging_Response),
  hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_member_array,  // message members
  hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_type_support_handle = {
  0,
  &hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hardware_monitor2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging_Response)() {
  if (!hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_type_support_handle.typesupport_identifier) {
    hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hardware_monitor2_interfaces__srv__Logging_Response__rosidl_typesupport_introspection_c__Logging_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hardware_monitor2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hardware_monitor2_interfaces/srv/detail/logging__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_members = {
  "hardware_monitor2_interfaces__srv",  // service namespace
  "Logging",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_Request_message_type_support_handle,
  NULL  // response message
  // hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_Response_message_type_support_handle
};

static rosidl_service_type_support_t hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_type_support_handle = {
  0,
  &hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hardware_monitor2_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging)() {
  if (!hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_type_support_handle.typesupport_identifier) {
    hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hardware_monitor2_interfaces, srv, Logging_Response)()->data;
  }

  return &hardware_monitor2_interfaces__srv__detail__logging__rosidl_typesupport_introspection_c__Logging_service_type_support_handle;
}
