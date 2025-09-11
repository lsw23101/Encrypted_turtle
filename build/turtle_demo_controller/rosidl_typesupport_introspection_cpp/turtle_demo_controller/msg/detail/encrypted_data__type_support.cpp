// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "turtle_demo_controller/msg/detail/encrypted_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace turtle_demo_controller
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void EncryptedData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) turtle_demo_controller::msg::EncryptedData(_init);
}

void EncryptedData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<turtle_demo_controller::msg::EncryptedData *>(message_memory);
  typed_message->~EncryptedData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember EncryptedData_message_member_array[2] = {
  {
    "encrypted_data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtle_demo_controller::msg::EncryptedData, encrypted_data),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(turtle_demo_controller::msg::EncryptedData, data_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers EncryptedData_message_members = {
  "turtle_demo_controller::msg",  // message namespace
  "EncryptedData",  // message name
  2,  // number of fields
  sizeof(turtle_demo_controller::msg::EncryptedData),
  EncryptedData_message_member_array,  // message members
  EncryptedData_init_function,  // function to initialize message memory (memory has to be allocated)
  EncryptedData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t EncryptedData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &EncryptedData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace turtle_demo_controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<turtle_demo_controller::msg::EncryptedData>()
{
  return &::turtle_demo_controller::msg::rosidl_typesupport_introspection_cpp::EncryptedData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, turtle_demo_controller, msg, EncryptedData)() {
  return &::turtle_demo_controller::msg::rosidl_typesupport_introspection_cpp::EncryptedData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
