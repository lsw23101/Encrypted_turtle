// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_H_
#define TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'encrypted_data'
// Member 'data_type'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/EncryptedData in the package turtle_demo_controller.
typedef struct turtle_demo_controller__msg__EncryptedData
{
  rosidl_runtime_c__String encrypted_data;
  rosidl_runtime_c__String data_type;
} turtle_demo_controller__msg__EncryptedData;

// Struct for a sequence of turtle_demo_controller__msg__EncryptedData.
typedef struct turtle_demo_controller__msg__EncryptedData__Sequence
{
  turtle_demo_controller__msg__EncryptedData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtle_demo_controller__msg__EncryptedData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLE_DEMO_CONTROLLER__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_H_
