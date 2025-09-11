// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__BUILDER_HPP_
#define TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__BUILDER_HPP_

#include "turtle_demo_controller/msg/detail/encrypted_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace turtle_demo_controller
{

namespace msg
{

namespace builder
{

class Init_EncryptedData_data_type
{
public:
  explicit Init_EncryptedData_data_type(::turtle_demo_controller::msg::EncryptedData & msg)
  : msg_(msg)
  {}
  ::turtle_demo_controller::msg::EncryptedData data_type(::turtle_demo_controller::msg::EncryptedData::_data_type_type arg)
  {
    msg_.data_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtle_demo_controller::msg::EncryptedData msg_;
};

class Init_EncryptedData_encrypted_data
{
public:
  Init_EncryptedData_encrypted_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncryptedData_data_type encrypted_data(::turtle_demo_controller::msg::EncryptedData::_encrypted_data_type arg)
  {
    msg_.encrypted_data = std::move(arg);
    return Init_EncryptedData_data_type(msg_);
  }

private:
  ::turtle_demo_controller::msg::EncryptedData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtle_demo_controller::msg::EncryptedData>()
{
  return turtle_demo_controller::msg::builder::Init_EncryptedData_encrypted_data();
}

}  // namespace turtle_demo_controller

#endif  // TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__BUILDER_HPP_
