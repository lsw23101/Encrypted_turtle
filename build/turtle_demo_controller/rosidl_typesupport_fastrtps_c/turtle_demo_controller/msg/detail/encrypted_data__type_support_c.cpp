// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice
#include "turtle_demo_controller/msg/detail/encrypted_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "turtle_demo_controller/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "turtle_demo_controller/msg/detail/encrypted_data__struct.h"
#include "turtle_demo_controller/msg/detail/encrypted_data__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // data_type, encrypted_data
#include "rosidl_runtime_c/string_functions.h"  // data_type, encrypted_data

// forward declare type support functions


using _EncryptedData__ros_msg_type = turtle_demo_controller__msg__EncryptedData;

static bool _EncryptedData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EncryptedData__ros_msg_type * ros_message = static_cast<const _EncryptedData__ros_msg_type *>(untyped_ros_message);
  // Field name: encrypted_data
  {
    const rosidl_runtime_c__String * str = &ros_message->encrypted_data;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: data_type
  {
    const rosidl_runtime_c__String * str = &ros_message->data_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _EncryptedData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EncryptedData__ros_msg_type * ros_message = static_cast<_EncryptedData__ros_msg_type *>(untyped_ros_message);
  // Field name: encrypted_data
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->encrypted_data.data) {
      rosidl_runtime_c__String__init(&ros_message->encrypted_data);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->encrypted_data,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'encrypted_data'\n");
      return false;
    }
  }

  // Field name: data_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->data_type.data) {
      rosidl_runtime_c__String__init(&ros_message->data_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->data_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'data_type'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtle_demo_controller
size_t get_serialized_size_turtle_demo_controller__msg__EncryptedData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EncryptedData__ros_msg_type * ros_message = static_cast<const _EncryptedData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name encrypted_data
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->encrypted_data.size + 1);
  // field.name data_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->data_type.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _EncryptedData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_turtle_demo_controller__msg__EncryptedData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_turtle_demo_controller
size_t max_serialized_size_turtle_demo_controller__msg__EncryptedData(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: encrypted_data
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: data_type
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

static size_t _EncryptedData__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_turtle_demo_controller__msg__EncryptedData(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_EncryptedData = {
  "turtle_demo_controller::msg",
  "EncryptedData",
  _EncryptedData__cdr_serialize,
  _EncryptedData__cdr_deserialize,
  _EncryptedData__get_serialized_size,
  _EncryptedData__max_serialized_size
};

static rosidl_message_type_support_t _EncryptedData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EncryptedData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, turtle_demo_controller, msg, EncryptedData)() {
  return &_EncryptedData__type_support;
}

#if defined(__cplusplus)
}
#endif
