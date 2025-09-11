// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from enc_turtle_cpp:msg/EncryptedData.idl
// generated code does not contain a copyright notice
#include "enc_turtle_cpp/msg/detail/encrypted_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
enc_turtle_cpp__msg__EncryptedData__init(enc_turtle_cpp__msg__EncryptedData * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    enc_turtle_cpp__msg__EncryptedData__fini(msg);
    return false;
  }
  // data_type
  return true;
}

void
enc_turtle_cpp__msg__EncryptedData__fini(enc_turtle_cpp__msg__EncryptedData * msg)
{
  if (!msg) {
    return;
  }
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
  // data_type
}

bool
enc_turtle_cpp__msg__EncryptedData__are_equal(const enc_turtle_cpp__msg__EncryptedData * lhs, const enc_turtle_cpp__msg__EncryptedData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // data_type
  if (lhs->data_type != rhs->data_type) {
    return false;
  }
  return true;
}

bool
enc_turtle_cpp__msg__EncryptedData__copy(
  const enc_turtle_cpp__msg__EncryptedData * input,
  enc_turtle_cpp__msg__EncryptedData * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // data_type
  output->data_type = input->data_type;
  return true;
}

enc_turtle_cpp__msg__EncryptedData *
enc_turtle_cpp__msg__EncryptedData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  enc_turtle_cpp__msg__EncryptedData * msg = (enc_turtle_cpp__msg__EncryptedData *)allocator.allocate(sizeof(enc_turtle_cpp__msg__EncryptedData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(enc_turtle_cpp__msg__EncryptedData));
  bool success = enc_turtle_cpp__msg__EncryptedData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
enc_turtle_cpp__msg__EncryptedData__destroy(enc_turtle_cpp__msg__EncryptedData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    enc_turtle_cpp__msg__EncryptedData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
enc_turtle_cpp__msg__EncryptedData__Sequence__init(enc_turtle_cpp__msg__EncryptedData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  enc_turtle_cpp__msg__EncryptedData * data = NULL;

  if (size) {
    data = (enc_turtle_cpp__msg__EncryptedData *)allocator.zero_allocate(size, sizeof(enc_turtle_cpp__msg__EncryptedData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = enc_turtle_cpp__msg__EncryptedData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        enc_turtle_cpp__msg__EncryptedData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
enc_turtle_cpp__msg__EncryptedData__Sequence__fini(enc_turtle_cpp__msg__EncryptedData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      enc_turtle_cpp__msg__EncryptedData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

enc_turtle_cpp__msg__EncryptedData__Sequence *
enc_turtle_cpp__msg__EncryptedData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  enc_turtle_cpp__msg__EncryptedData__Sequence * array = (enc_turtle_cpp__msg__EncryptedData__Sequence *)allocator.allocate(sizeof(enc_turtle_cpp__msg__EncryptedData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = enc_turtle_cpp__msg__EncryptedData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
enc_turtle_cpp__msg__EncryptedData__Sequence__destroy(enc_turtle_cpp__msg__EncryptedData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    enc_turtle_cpp__msg__EncryptedData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
enc_turtle_cpp__msg__EncryptedData__Sequence__are_equal(const enc_turtle_cpp__msg__EncryptedData__Sequence * lhs, const enc_turtle_cpp__msg__EncryptedData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!enc_turtle_cpp__msg__EncryptedData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
enc_turtle_cpp__msg__EncryptedData__Sequence__copy(
  const enc_turtle_cpp__msg__EncryptedData__Sequence * input,
  enc_turtle_cpp__msg__EncryptedData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(enc_turtle_cpp__msg__EncryptedData);
    enc_turtle_cpp__msg__EncryptedData * data =
      (enc_turtle_cpp__msg__EncryptedData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!enc_turtle_cpp__msg__EncryptedData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          enc_turtle_cpp__msg__EncryptedData__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!enc_turtle_cpp__msg__EncryptedData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
