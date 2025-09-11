// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__TRAITS_HPP_
#define TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__TRAITS_HPP_

#include "turtle_demo_controller/msg/detail/encrypted_data__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtle_demo_controller::msg::EncryptedData>()
{
  return "turtle_demo_controller::msg::EncryptedData";
}

template<>
inline const char * name<turtle_demo_controller::msg::EncryptedData>()
{
  return "turtle_demo_controller/msg/EncryptedData";
}

template<>
struct has_fixed_size<turtle_demo_controller::msg::EncryptedData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtle_demo_controller::msg::EncryptedData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtle_demo_controller::msg::EncryptedData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__TRAITS_HPP_
