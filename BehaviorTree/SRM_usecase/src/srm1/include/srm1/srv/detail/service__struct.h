// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from srm1:srv/Service.idl
// generated code does not contain a copyright notice

#ifndef SRM1__SRV__DETAIL__SERVICE__STRUCT_H_
#define SRM1__SRV__DETAIL__SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Service in the package srm1.
typedef struct srm1__srv__Service_Request
{
  float coordinates[4];
  float distance;
} srm1__srv__Service_Request;

// Struct for a sequence of srm1__srv__Service_Request.
typedef struct srm1__srv__Service_Request__Sequence
{
  srm1__srv__Service_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__srv__Service_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Service in the package srm1.
typedef struct srm1__srv__Service_Response
{
  uint8_t structure_needs_at_least_one_member;
} srm1__srv__Service_Response;

// Struct for a sequence of srm1__srv__Service_Response.
typedef struct srm1__srv__Service_Response__Sequence
{
  srm1__srv__Service_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__srv__Service_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SRM1__SRV__DETAIL__SERVICE__STRUCT_H_
