// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__BUILDER_HPP_
#define ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__BUILDER_HPP_

#include "enc_turtle_cpp/msg/detail/encrypted_data__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace enc_turtle_cpp
{

namespace msg
{

namespace builder
{

class Init_EncryptedData_data_type
{
public:
  explicit Init_EncryptedData_data_type(::enc_turtle_cpp::msg::EncryptedData & msg)
  : msg_(msg)
  {}
  ::enc_turtle_cpp::msg::EncryptedData data_type(::enc_turtle_cpp::msg::EncryptedData::_data_type_type arg)
  {
    msg_.data_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::enc_turtle_cpp::msg::EncryptedData msg_;
};

class Init_EncryptedData_data
{
public:
  Init_EncryptedData_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncryptedData_data_type data(::enc_turtle_cpp::msg::EncryptedData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_EncryptedData_data_type(msg_);
  }

private:
  ::enc_turtle_cpp::msg::EncryptedData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::enc_turtle_cpp::msg::EncryptedData>()
{
  return enc_turtle_cpp::msg::builder::Init_EncryptedData_data();
}

}  // namespace enc_turtle_cpp

#endif  // ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__BUILDER_HPP_
