// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_H_
#define ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/EncryptedData in the package enc_turtle_cpp.
typedef struct enc_turtle_cpp__msg__EncryptedData
{
  rosidl_runtime_c__uint8__Sequence data;
  uint32_t data_type;
} enc_turtle_cpp__msg__EncryptedData;

// Struct for a sequence of enc_turtle_cpp__msg__EncryptedData.
typedef struct enc_turtle_cpp__msg__EncryptedData__Sequence
{
  enc_turtle_cpp__msg__EncryptedData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} enc_turtle_cpp__msg__EncryptedData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__STRUCT_H_
