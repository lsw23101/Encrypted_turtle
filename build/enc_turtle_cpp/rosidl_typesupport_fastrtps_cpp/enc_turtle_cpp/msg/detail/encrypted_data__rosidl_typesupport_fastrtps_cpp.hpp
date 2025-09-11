// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "enc_turtle_cpp/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "enc_turtle_cpp/msg/detail/encrypted_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace enc_turtle_cpp
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
cdr_serialize(
  const enc_turtle_cpp::msg::EncryptedData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  enc_turtle_cpp::msg::EncryptedData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
get_serialized_size(
  const enc_turtle_cpp::msg::EncryptedData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
max_serialized_size_EncryptedData(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace enc_turtle_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, enc_turtle_cpp, msg, EncryptedData)();

#ifdef __cplusplus
}
#endif

#endif  // ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
