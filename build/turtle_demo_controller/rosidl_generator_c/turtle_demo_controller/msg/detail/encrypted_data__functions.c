// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtle_demo_controller:msg/EncryptedData.idl
// generated code does not contain a copyright notice
#include "turtle_demo_controller/msg/detail/encrypted_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `encrypted_data`
// Member `data_type`
#include "rosidl_runtime_c/string_functions.h"

bool
turtle_demo_controller__msg__EncryptedData__init(turtle_demo_controller__msg__EncryptedData * msg)
{
  if (!msg) {
    return false;
  }
  // encrypted_data
  if (!rosidl_runtime_c__String__init(&msg->encrypted_data)) {
    turtle_demo_controller__msg__EncryptedData__fini(msg);
    return false;
  }
  // data_type
  if (!rosidl_runtime_c__String__init(&msg->data_type)) {
    turtle_demo_controller__msg__EncryptedData__fini(msg);
    return false;
  }
  return true;
}

void
turtle_demo_controller__msg__EncryptedData__fini(turtle_demo_controller__msg__EncryptedData * msg)
{
  if (!msg) {
    return;
  }
  // encrypted_data
  rosidl_runtime_c__String__fini(&msg->encrypted_data);
  // data_type
  rosidl_runtime_c__String__fini(&msg->data_type);
}

bool
turtle_demo_controller__msg__EncryptedData__are_equal(const turtle_demo_controller__msg__EncryptedData * lhs, const turtle_demo_controller__msg__EncryptedData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // encrypted_data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->encrypted_data), &(rhs->encrypted_data)))
  {
    return false;
  }
  // data_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data_type), &(rhs->data_type)))
  {
    return false;
  }
  return true;
}

bool
turtle_demo_controller__msg__EncryptedData__copy(
  const turtle_demo_controller__msg__EncryptedData * input,
  turtle_demo_controller__msg__EncryptedData * output)
{
  if (!input || !output) {
    return false;
  }
  // encrypted_data
  if (!rosidl_runtime_c__String__copy(
      &(input->encrypted_data), &(output->encrypted_data)))
  {
    return false;
  }
  // data_type
  if (!rosidl_runtime_c__String__copy(
      &(input->data_type), &(output->data_type)))
  {
    return false;
  }
  return true;
}

turtle_demo_controller__msg__EncryptedData *
turtle_demo_controller__msg__EncryptedData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtle_demo_controller__msg__EncryptedData * msg = (turtle_demo_controller__msg__EncryptedData *)allocator.allocate(sizeof(turtle_demo_controller__msg__EncryptedData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtle_demo_controller__msg__EncryptedData));
  bool success = turtle_demo_controller__msg__EncryptedData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtle_demo_controller__msg__EncryptedData__destroy(turtle_demo_controller__msg__EncryptedData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtle_demo_controller__msg__EncryptedData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtle_demo_controller__msg__EncryptedData__Sequence__init(turtle_demo_controller__msg__EncryptedData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtle_demo_controller__msg__EncryptedData * data = NULL;

  if (size) {
    data = (turtle_demo_controller__msg__EncryptedData *)allocator.zero_allocate(size, sizeof(turtle_demo_controller__msg__EncryptedData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtle_demo_controller__msg__EncryptedData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtle_demo_controller__msg__EncryptedData__fini(&data[i - 1]);
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
turtle_demo_controller__msg__EncryptedData__Sequence__fini(turtle_demo_controller__msg__EncryptedData__Sequence * array)
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
      turtle_demo_controller__msg__EncryptedData__fini(&array->data[i]);
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

turtle_demo_controller__msg__EncryptedData__Sequence *
turtle_demo_controller__msg__EncryptedData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtle_demo_controller__msg__EncryptedData__Sequence * array = (turtle_demo_controller__msg__EncryptedData__Sequence *)allocator.allocate(sizeof(turtle_demo_controller__msg__EncryptedData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtle_demo_controller__msg__EncryptedData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtle_demo_controller__msg__EncryptedData__Sequence__destroy(turtle_demo_controller__msg__EncryptedData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtle_demo_controller__msg__EncryptedData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtle_demo_controller__msg__EncryptedData__Sequence__are_equal(const turtle_demo_controller__msg__EncryptedData__Sequence * lhs, const turtle_demo_controller__msg__EncryptedData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtle_demo_controller__msg__EncryptedData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtle_demo_controller__msg__EncryptedData__Sequence__copy(
  const turtle_demo_controller__msg__EncryptedData__Sequence * input,
  turtle_demo_controller__msg__EncryptedData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtle_demo_controller__msg__EncryptedData);
    turtle_demo_controller__msg__EncryptedData * data =
      (turtle_demo_controller__msg__EncryptedData *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtle_demo_controller__msg__EncryptedData__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          turtle_demo_controller__msg__EncryptedData__fini(&data[i]);
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
    if (!turtle_demo_controller__msg__EncryptedData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
