// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice
#include "turtle_demo_controller/msg/detail/encrypted_data__rosidl_typesupport_fastrtps_cpp.hpp"
#include "turtle_demo_controller/msg/detail/encrypted_data__struct.hpp"

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

namespace turtle_demo_controller
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtle_demo_controller
cdr_serialize(
  const turtle_demo_controller::msg::EncryptedData & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: encrypted_data
  cdr << ros_message.encrypted_data;
  // Member: data_type
  cdr << ros_message.data_type;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtle_demo_controller
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  turtle_demo_controller::msg::EncryptedData & ros_message)
{
  // Member: encrypted_data
  cdr >> ros_message.encrypted_data;

  // Member: data_type
  cdr >> ros_message.data_type;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtle_demo_controller
get_serialized_size(
  const turtle_demo_controller::msg::EncryptedData & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: encrypted_data
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.encrypted_data.size() + 1);
  // Member: data_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.data_type.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtle_demo_controller
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


  // Member: encrypted_data
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: data_type
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _EncryptedData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const turtle_demo_controller::msg::EncryptedData *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EncryptedData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<turtle_demo_controller::msg::EncryptedData *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EncryptedData__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const turtle_demo_controller::msg::EncryptedData *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EncryptedData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_EncryptedData(full_bounded, 0);
}

static message_type_support_callbacks_t _EncryptedData__callbacks = {
  "turtle_demo_controller::msg",
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

}  // namespace turtle_demo_controller

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_turtle_demo_controller
const rosidl_message_type_support_t *
get_message_type_support_handle<turtle_demo_controller::msg::EncryptedData>()
{
  return &turtle_demo_controller::msg::typesupport_fastrtps_cpp::_EncryptedData__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtle_demo_controller, msg, EncryptedData)() {
  return &turtle_demo_controller::msg::typesupport_fastrtps_cpp::_EncryptedData__handle;
}

#ifdef __cplusplus
}
#endif
