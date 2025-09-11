// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_HPP_
#define TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__turtle_demo_controller__msg__EncryptedData __attribute__((deprecated))
#else
# define DEPRECATED__turtle_demo_controller__msg__EncryptedData __declspec(deprecated)
#endif

namespace turtle_demo_controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncryptedData_
{
  using Type = EncryptedData_<ContainerAllocator>;

  explicit EncryptedData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encrypted_data = "";
      this->data_type = "";
    }
  }

  explicit EncryptedData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : encrypted_data(_alloc),
    data_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encrypted_data = "";
      this->data_type = "";
    }
  }

  // field types and members
  using _encrypted_data_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _encrypted_data_type encrypted_data;
  using _data_type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _data_type_type data_type;

  // setters for named parameter idiom
  Type & set__encrypted_data(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->encrypted_data = _arg;
    return *this;
  }
  Type & set__data_type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->data_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtle_demo_controller::msg::EncryptedData_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtle_demo_controller::msg::EncryptedData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtle_demo_controller::msg::EncryptedData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtle_demo_controller::msg::EncryptedData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtle_demo_controller__msg__EncryptedData
    std::shared_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtle_demo_controller__msg__EncryptedData
    std::shared_ptr<turtle_demo_controller::msg::EncryptedData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncryptedData_ & other) const
  {
    if (this->encrypted_data != other.encrypted_data) {
      return false;
    }
    if (this->data_type != other.data_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncryptedData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncryptedData_

// alias to use template instance with default allocator
using EncryptedData =
  turtle_demo_controller::msg::EncryptedData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace turtle_demo_controller

#endif  // TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_HPP_
