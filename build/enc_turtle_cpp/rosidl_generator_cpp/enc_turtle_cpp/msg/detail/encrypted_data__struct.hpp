// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_HPP_
#define ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__enc_turtle_cpp__msg__EncryptedData __attribute__((deprecated))
#else
# define DEPRECATED__enc_turtle_cpp__msg__EncryptedData __declspec(deprecated)
#endif

namespace enc_turtle_cpp
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
      this->data_type = 0ul;
    }
  }

  explicit EncryptedData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data_type = 0ul;
    }
  }

  // field types and members
  using _data_type =
    std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other>;
  _data_type data;
  using _data_type_type =
    uint32_t;
  _data_type_type data_type;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__data_type(
    const uint32_t & _arg)
  {
    this->data_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator> *;
  using ConstRawPtr =
    const enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__enc_turtle_cpp__msg__EncryptedData
    std::shared_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__enc_turtle_cpp__msg__EncryptedData
    std::shared_ptr<enc_turtle_cpp::msg::EncryptedData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncryptedData_ & other) const
  {
    if (this->data != other.data) {
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
  enc_turtle_cpp::msg::EncryptedData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace enc_turtle_cpp

#endif  // ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_HPP_
