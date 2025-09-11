// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice
#include "enc_turtle_cpp/msg/detail/encrypted_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "enc_turtle_cpp/msg/detail/encrypted_data__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: data
  {
    cdr << ros_message.data;
  }
  // Member: data_type
  cdr << ros_message.data_type;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  enc_turtle_cpp::msg::EncryptedData & ros_message)
{
  // Member: data
  {
    cdr >> ros_message.data;
  }

  // Member: data_type
  cdr >> ros_message.data_type;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
get_serialized_size(
  const enc_turtle_cpp::msg::EncryptedData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: data
  {
    size_t array_size = ros_message.data.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.data[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: data_type
  {
    size_t item_size = sizeof(ros_message.data_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_enc_turtle_cpp
max_serialized_size_EncryptedData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: data
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: data_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _EncryptedData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const enc_turtle_cpp::msg::EncryptedData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EncryptedData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<enc_turtle_cpp::msg::EncryptedData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EncryptedData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const enc_turtle_cpp::msg::EncryptedData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EncryptedData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_EncryptedData(full_bounded, 0);
}

static message_type_support_callbacks_t _EncryptedData__callbacks = {
  "enc_turtle_cpp::msg",
  "EncryptedData",
  _EncryptedData__cdr_serialize,
  _EncryptedData__cdr_deserialize,
  _EncryptedData__get_serialized_size,
  _EncryptedData__max_serialized_size
};

static rosidl_message_type_support_t _EncryptedData__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EncryptedData__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace enc_turtle_cpp

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_enc_turtle_cpp
const rosidl_message_type_support_t *
get_message_type_support_handle<enc_turtle_cpp::msg::EncryptedData>()
{
  return &enc_turtle_cpp::msg::typesupport_fastrtps_cpp::_EncryptedData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, enc_turtle_cpp, msg, EncryptedData)() {
  return &enc_turtle_cpp::msg::typesupport_fastrtps_cpp::_EncryptedData__handle;
}

#ifdef __cplusplus
}
#endif
