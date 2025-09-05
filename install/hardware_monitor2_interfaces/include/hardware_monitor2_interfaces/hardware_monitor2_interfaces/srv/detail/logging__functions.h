// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice

#ifndef HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__FUNCTIONS_H_
#define HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hardware_monitor2_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "hardware_monitor2_interfaces/srv/detail/logging__struct.h"

/// Initialize srv/Logging message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hardware_monitor2_interfaces__srv__Logging_Request
 * )) before or use
 * hardware_monitor2_interfaces__srv__Logging_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Request__init(hardware_monitor2_interfaces__srv__Logging_Request * msg);

/// Finalize srv/Logging message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Request__fini(hardware_monitor2_interfaces__srv__Logging_Request * msg);

/// Create srv/Logging message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hardware_monitor2_interfaces__srv__Logging_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
hardware_monitor2_interfaces__srv__Logging_Request *
hardware_monitor2_interfaces__srv__Logging_Request__create();

/// Destroy srv/Logging message.
/**
 * It calls
 * hardware_monitor2_interfaces__srv__Logging_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Request__destroy(hardware_monitor2_interfaces__srv__Logging_Request * msg);

/// Check for srv/Logging message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Request__are_equal(const hardware_monitor2_interfaces__srv__Logging_Request * lhs, const hardware_monitor2_interfaces__srv__Logging_Request * rhs);

/// Copy a srv/Logging message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Request__copy(
  const hardware_monitor2_interfaces__srv__Logging_Request * input,
  hardware_monitor2_interfaces__srv__Logging_Request * output);

/// Initialize array of srv/Logging messages.
/**
 * It allocates the memory for the number of elements and calls
 * hardware_monitor2_interfaces__srv__Logging_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__init(hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array, size_t size);

/// Finalize array of srv/Logging messages.
/**
 * It calls
 * hardware_monitor2_interfaces__srv__Logging_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__fini(hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array);

/// Create array of srv/Logging messages.
/**
 * It allocates the memory for the array and calls
 * hardware_monitor2_interfaces__srv__Logging_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
hardware_monitor2_interfaces__srv__Logging_Request__Sequence *
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__create(size_t size);

/// Destroy array of srv/Logging messages.
/**
 * It calls
 * hardware_monitor2_interfaces__srv__Logging_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__destroy(hardware_monitor2_interfaces__srv__Logging_Request__Sequence * array);

/// Check for srv/Logging message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__are_equal(const hardware_monitor2_interfaces__srv__Logging_Request__Sequence * lhs, const hardware_monitor2_interfaces__srv__Logging_Request__Sequence * rhs);

/// Copy an array of srv/Logging messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Request__Sequence__copy(
  const hardware_monitor2_interfaces__srv__Logging_Request__Sequence * input,
  hardware_monitor2_interfaces__srv__Logging_Request__Sequence * output);

/// Initialize srv/Logging message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hardware_monitor2_interfaces__srv__Logging_Response
 * )) before or use
 * hardware_monitor2_interfaces__srv__Logging_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Response__init(hardware_monitor2_interfaces__srv__Logging_Response * msg);

/// Finalize srv/Logging message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Response__fini(hardware_monitor2_interfaces__srv__Logging_Response * msg);

/// Create srv/Logging message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hardware_monitor2_interfaces__srv__Logging_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
hardware_monitor2_interfaces__srv__Logging_Response *
hardware_monitor2_interfaces__srv__Logging_Response__create();

/// Destroy srv/Logging message.
/**
 * It calls
 * hardware_monitor2_interfaces__srv__Logging_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Response__destroy(hardware_monitor2_interfaces__srv__Logging_Response * msg);

/// Check for srv/Logging message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Response__are_equal(const hardware_monitor2_interfaces__srv__Logging_Response * lhs, const hardware_monitor2_interfaces__srv__Logging_Response * rhs);

/// Copy a srv/Logging message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Response__copy(
  const hardware_monitor2_interfaces__srv__Logging_Response * input,
  hardware_monitor2_interfaces__srv__Logging_Response * output);

/// Initialize array of srv/Logging messages.
/**
 * It allocates the memory for the number of elements and calls
 * hardware_monitor2_interfaces__srv__Logging_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__init(hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array, size_t size);

/// Finalize array of srv/Logging messages.
/**
 * It calls
 * hardware_monitor2_interfaces__srv__Logging_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__fini(hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array);

/// Create array of srv/Logging messages.
/**
 * It allocates the memory for the array and calls
 * hardware_monitor2_interfaces__srv__Logging_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
hardware_monitor2_interfaces__srv__Logging_Response__Sequence *
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__create(size_t size);

/// Destroy array of srv/Logging messages.
/**
 * It calls
 * hardware_monitor2_interfaces__srv__Logging_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
void
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__destroy(hardware_monitor2_interfaces__srv__Logging_Response__Sequence * array);

/// Check for srv/Logging message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__are_equal(const hardware_monitor2_interfaces__srv__Logging_Response__Sequence * lhs, const hardware_monitor2_interfaces__srv__Logging_Response__Sequence * rhs);

/// Copy an array of srv/Logging messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hardware_monitor2_interfaces
bool
hardware_monitor2_interfaces__srv__Logging_Response__Sequence__copy(
  const hardware_monitor2_interfaces__srv__Logging_Response__Sequence * input,
  hardware_monitor2_interfaces__srv__Logging_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__FUNCTIONS_H_
