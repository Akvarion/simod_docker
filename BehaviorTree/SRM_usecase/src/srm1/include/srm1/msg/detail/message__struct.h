// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from srm1:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef SRM1__MSG__DETAIL__MESSAGE__STRUCT_H_
#define SRM1__MSG__DETAIL__MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Message in the package srm1.
typedef struct srm1__msg__Message
{
  rosidl_runtime_c__String message;
} srm1__msg__Message;

// Struct for a sequence of srm1__msg__Message.
typedef struct srm1__msg__Message__Sequence
{
  srm1__msg__Message * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__msg__Message__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SRM1__MSG__DETAIL__MESSAGE__STRUCT_H_
