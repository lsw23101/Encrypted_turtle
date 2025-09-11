// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice

#ifndef ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__FUNCTIONS_H_
#define ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "enc_turtle_cpp/msg/rosidl_generator_c__visibility_control.h"

#include "enc_turtle_cpp/msg/detail/encrypted_data__struct.h"

/// Initialize msg/EncryptedData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * enc_turtle_cpp__msg__EncryptedData
 * )) before or use
 * enc_turtle_cpp__msg__EncryptedData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
bool
enc_turtle_cpp__msg__EncryptedData__init(enc_turtle_cpp__msg__EncryptedData * msg);

/// Finalize msg/EncryptedData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
void
enc_turtle_cpp__msg__EncryptedData__fini(enc_turtle_cpp__msg__EncryptedData * msg);

/// Create msg/EncryptedData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * enc_turtle_cpp__msg__EncryptedData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
enc_turtle_cpp__msg__EncryptedData *
enc_turtle_cpp__msg__EncryptedData__create();

/// Destroy msg/EncryptedData message.
/**
 * It calls
 * enc_turtle_cpp__msg__EncryptedData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
void
enc_turtle_cpp__msg__EncryptedData__destroy(enc_turtle_cpp__msg__EncryptedData * msg);

/// Check for msg/EncryptedData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
bool
enc_turtle_cpp__msg__EncryptedData__are_equal(const enc_turtle_cpp__msg__EncryptedData * lhs, const enc_turtle_cpp__msg__EncryptedData * rhs);

/// Copy a msg/EncryptedData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
bool
enc_turtle_cpp__msg__EncryptedData__copy(
  const enc_turtle_cpp__msg__EncryptedData * input,
  enc_turtle_cpp__msg__EncryptedData * output);

/// Initialize array of msg/EncryptedData messages.
/**
 * It allocates the memory for the number of elements and calls
 * enc_turtle_cpp__msg__EncryptedData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
bool
enc_turtle_cpp__msg__EncryptedData__Sequence__init(enc_turtle_cpp__msg__EncryptedData__Sequence * array, size_t size);

/// Finalize array of msg/EncryptedData messages.
/**
 * It calls
 * enc_turtle_cpp__msg__EncryptedData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
void
enc_turtle_cpp__msg__EncryptedData__Sequence__fini(enc_turtle_cpp__msg__EncryptedData__Sequence * array);

/// Create array of msg/EncryptedData messages.
/**
 * It allocates the memory for the array and calls
 * enc_turtle_cpp__msg__EncryptedData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
enc_turtle_cpp__msg__EncryptedData__Sequence *
enc_turtle_cpp__msg__EncryptedData__Sequence__create(size_t size);

/// Destroy array of msg/EncryptedData messages.
/**
 * It calls
 * enc_turtle_cpp__msg__EncryptedData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
void
enc_turtle_cpp__msg__EncryptedData__Sequence__destroy(enc_turtle_cpp__msg__EncryptedData__Sequence * array);

/// Check for msg/EncryptedData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
bool
enc_turtle_cpp__msg__EncryptedData__Sequence__are_equal(const enc_turtle_cpp__msg__EncryptedData__Sequence * lhs, const enc_turtle_cpp__msg__EncryptedData__Sequence * rhs);

/// Copy an array of msg/EncryptedData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_enc_turtle_cpp
bool
enc_turtle_cpp__msg__EncryptedData__Sequence__copy(
  const enc_turtle_cpp__msg__EncryptedData__Sequence * input,
  enc_turtle_cpp__msg__EncryptedData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ENC_TURTLE_CPP__MSG__DETAIL__ENCRYPTED_DATA__FUNCTIONS_H_
