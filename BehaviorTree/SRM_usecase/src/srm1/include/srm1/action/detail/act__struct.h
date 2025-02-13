// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from srm1:action/Act.idl
// generated code does not contain a copyright notice

#ifndef SRM1__ACTION__DETAIL__ACT__STRUCT_H_
#define SRM1__ACTION__DETAIL__ACT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'request'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_Goal
{
  rosidl_runtime_c__String request;
} srm1__action__Act_Goal;

// Struct for a sequence of srm1__action__Act_Goal.
typedef struct srm1__action__Act_Goal__Sequence
{
  srm1__action__Act_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_Result
{
  rosidl_runtime_c__String response;
} srm1__action__Act_Result;

// Struct for a sequence of srm1__action__Act_Result.
typedef struct srm1__action__Act_Result__Sequence
{
  srm1__action__Act_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_Feedback
{
  rosidl_runtime_c__String feedback;
} srm1__action__Act_Feedback;

// Struct for a sequence of srm1__action__Act_Feedback.
typedef struct srm1__action__Act_Feedback__Sequence
{
  srm1__action__Act_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "srm1/action/detail/act__struct.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  srm1__action__Act_Goal goal;
} srm1__action__Act_SendGoal_Request;

// Struct for a sequence of srm1__action__Act_SendGoal_Request.
typedef struct srm1__action__Act_SendGoal_Request__Sequence
{
  srm1__action__Act_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} srm1__action__Act_SendGoal_Response;

// Struct for a sequence of srm1__action__Act_SendGoal_Response.
typedef struct srm1__action__Act_SendGoal_Response__Sequence
{
  srm1__action__Act_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} srm1__action__Act_GetResult_Request;

// Struct for a sequence of srm1__action__Act_GetResult_Request.
typedef struct srm1__action__Act_GetResult_Request__Sequence
{
  srm1__action__Act_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "srm1/action/detail/act__struct.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_GetResult_Response
{
  int8_t status;
  srm1__action__Act_Result result;
} srm1__action__Act_GetResult_Response;

// Struct for a sequence of srm1__action__Act_GetResult_Response.
typedef struct srm1__action__Act_GetResult_Response__Sequence
{
  srm1__action__Act_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "srm1/action/detail/act__struct.h"

/// Struct defined in action/Act in the package srm1.
typedef struct srm1__action__Act_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  srm1__action__Act_Feedback feedback;
} srm1__action__Act_FeedbackMessage;

// Struct for a sequence of srm1__action__Act_FeedbackMessage.
typedef struct srm1__action__Act_FeedbackMessage__Sequence
{
  srm1__action__Act_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} srm1__action__Act_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SRM1__ACTION__DETAIL__ACT__STRUCT_H_
