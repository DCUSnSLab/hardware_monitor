// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hardware_monitor2_interfaces:srv/Logging.idl
// generated code does not contain a copyright notice

#ifndef HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__STRUCT_HPP_
#define HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Request __attribute__((deprecated))
#else
# define DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Request __declspec(deprecated)
#endif

namespace hardware_monitor2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Logging_Request_
{
  using Type = Logging_Request_<ContainerAllocator>;

  explicit Logging_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_logging = "";
    }
  }

  explicit Logging_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : is_logging(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_logging = "";
    }
  }

  // field types and members
  using _is_logging_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _is_logging_type is_logging;

  // setters for named parameter idiom
  Type & set__is_logging(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->is_logging = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Request
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Request
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Logging_Request_ & other) const
  {
    if (this->is_logging != other.is_logging) {
      return false;
    }
    return true;
  }
  bool operator!=(const Logging_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Logging_Request_

// alias to use template instance with default allocator
using Logging_Request =
  hardware_monitor2_interfaces::srv::Logging_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hardware_monitor2_interfaces


#ifndef _WIN32
# define DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Response __attribute__((deprecated))
#else
# define DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Response __declspec(deprecated)
#endif

namespace hardware_monitor2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Logging_Response_
{
  using Type = Logging_Response_<ContainerAllocator>;

  explicit Logging_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->logging_status = "";
    }
  }

  explicit Logging_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : logging_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->logging_status = "";
    }
  }

  // field types and members
  using _logging_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _logging_status_type logging_status;

  // setters for named parameter idiom
  Type & set__logging_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->logging_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Response
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hardware_monitor2_interfaces__srv__Logging_Response
    std::shared_ptr<hardware_monitor2_interfaces::srv::Logging_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Logging_Response_ & other) const
  {
    if (this->logging_status != other.logging_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const Logging_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Logging_Response_

// alias to use template instance with default allocator
using Logging_Response =
  hardware_monitor2_interfaces::srv::Logging_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

namespace hardware_monitor2_interfaces
{

namespace srv
{

struct Logging
{
  using Request = hardware_monitor2_interfaces::srv::Logging_Request;
  using Response = hardware_monitor2_interfaces::srv::Logging_Response;
};

}  // namespace srv

}  // namespace hardware_monitor2_interfaces

#endif  // HARDWARE_MONITOR2_INTERFACES__SRV__DETAIL__LOGGING__STRUCT_HPP_
