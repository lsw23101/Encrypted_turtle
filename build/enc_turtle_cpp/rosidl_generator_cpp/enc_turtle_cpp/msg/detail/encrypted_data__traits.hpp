// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__TRAITS_HPP_
#define ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__TRAITS_HPP_

#include "enc_turtle_cpp/msg/detail/encrypted_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<enc_turtle_cpp::msg::EncryptedData>()
{
  return "enc_turtle_cpp::msg::EncryptedData";
}

template<>
inline const char * name<enc_turtle_cpp::msg::EncryptedData>()
{
  return "enc_turtle_cpp/msg/EncryptedData";
}

template<>
struct has_fixed_size<enc_turtle_cpp::msg::EncryptedData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<enc_turtle_cpp::msg::EncryptedData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<enc_turtle_cpp::msg::EncryptedData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__TRAITS_HPP_
