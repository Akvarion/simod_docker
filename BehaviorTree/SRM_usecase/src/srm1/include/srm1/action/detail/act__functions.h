// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from srm1:action/Act.idl
// generated code does not contain a copyright notice

#ifndef SRM1__ACTION__DETAIL__ACT__FUNCTIONS_H_
#define SRM1__ACTION__DETAIL__ACT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "srm1/msg/rosidl_generator_c__visibility_control.h"

#include "srm1/action/detail/act__struct.h"

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_Goal
 * )) before or use
 * srm1__action__Act_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Goal__init(srm1__action__Act_Goal * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Goal__fini(srm1__action__Act_Goal * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_Goal *
srm1__action__Act_Goal__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Goal__destroy(srm1__action__Act_Goal * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Goal__are_equal(const srm1__action__Act_Goal * lhs, const srm1__action__Act_Goal * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Goal__copy(
  const srm1__action__Act_Goal * input,
  srm1__action__Act_Goal * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Goal__Sequence__init(srm1__action__Act_Goal__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Goal__Sequence__fini(srm1__action__Act_Goal__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_Goal__Sequence *
srm1__action__Act_Goal__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Goal__Sequence__destroy(srm1__action__Act_Goal__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Goal__Sequence__are_equal(const srm1__action__Act_Goal__Sequence * lhs, const srm1__action__Act_Goal__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Goal__Sequence__copy(
  const srm1__action__Act_Goal__Sequence * input,
  srm1__action__Act_Goal__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_Result
 * )) before or use
 * srm1__action__Act_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Result__init(srm1__action__Act_Result * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Result__fini(srm1__action__Act_Result * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_Result *
srm1__action__Act_Result__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Result__destroy(srm1__action__Act_Result * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Result__are_equal(const srm1__action__Act_Result * lhs, const srm1__action__Act_Result * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Result__copy(
  const srm1__action__Act_Result * input,
  srm1__action__Act_Result * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Result__Sequence__init(srm1__action__Act_Result__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Result__Sequence__fini(srm1__action__Act_Result__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_Result__Sequence *
srm1__action__Act_Result__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Result__Sequence__destroy(srm1__action__Act_Result__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Result__Sequence__are_equal(const srm1__action__Act_Result__Sequence * lhs, const srm1__action__Act_Result__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Result__Sequence__copy(
  const srm1__action__Act_Result__Sequence * input,
  srm1__action__Act_Result__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_Feedback
 * )) before or use
 * srm1__action__Act_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Feedback__init(srm1__action__Act_Feedback * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Feedback__fini(srm1__action__Act_Feedback * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_Feedback *
srm1__action__Act_Feedback__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Feedback__destroy(srm1__action__Act_Feedback * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Feedback__are_equal(const srm1__action__Act_Feedback * lhs, const srm1__action__Act_Feedback * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Feedback__copy(
  const srm1__action__Act_Feedback * input,
  srm1__action__Act_Feedback * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Feedback__Sequence__init(srm1__action__Act_Feedback__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Feedback__Sequence__fini(srm1__action__Act_Feedback__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_Feedback__Sequence *
srm1__action__Act_Feedback__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_Feedback__Sequence__destroy(srm1__action__Act_Feedback__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Feedback__Sequence__are_equal(const srm1__action__Act_Feedback__Sequence * lhs, const srm1__action__Act_Feedback__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_Feedback__Sequence__copy(
  const srm1__action__Act_Feedback__Sequence * input,
  srm1__action__Act_Feedback__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_SendGoal_Request
 * )) before or use
 * srm1__action__Act_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Request__init(srm1__action__Act_SendGoal_Request * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Request__fini(srm1__action__Act_SendGoal_Request * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_SendGoal_Request *
srm1__action__Act_SendGoal_Request__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Request__destroy(srm1__action__Act_SendGoal_Request * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Request__are_equal(const srm1__action__Act_SendGoal_Request * lhs, const srm1__action__Act_SendGoal_Request * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Request__copy(
  const srm1__action__Act_SendGoal_Request * input,
  srm1__action__Act_SendGoal_Request * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Request__Sequence__init(srm1__action__Act_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Request__Sequence__fini(srm1__action__Act_SendGoal_Request__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_SendGoal_Request__Sequence *
srm1__action__Act_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Request__Sequence__destroy(srm1__action__Act_SendGoal_Request__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Request__Sequence__are_equal(const srm1__action__Act_SendGoal_Request__Sequence * lhs, const srm1__action__Act_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Request__Sequence__copy(
  const srm1__action__Act_SendGoal_Request__Sequence * input,
  srm1__action__Act_SendGoal_Request__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_SendGoal_Response
 * )) before or use
 * srm1__action__Act_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Response__init(srm1__action__Act_SendGoal_Response * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Response__fini(srm1__action__Act_SendGoal_Response * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_SendGoal_Response *
srm1__action__Act_SendGoal_Response__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Response__destroy(srm1__action__Act_SendGoal_Response * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Response__are_equal(const srm1__action__Act_SendGoal_Response * lhs, const srm1__action__Act_SendGoal_Response * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Response__copy(
  const srm1__action__Act_SendGoal_Response * input,
  srm1__action__Act_SendGoal_Response * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Response__Sequence__init(srm1__action__Act_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Response__Sequence__fini(srm1__action__Act_SendGoal_Response__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_SendGoal_Response__Sequence *
srm1__action__Act_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_SendGoal_Response__Sequence__destroy(srm1__action__Act_SendGoal_Response__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Response__Sequence__are_equal(const srm1__action__Act_SendGoal_Response__Sequence * lhs, const srm1__action__Act_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_SendGoal_Response__Sequence__copy(
  const srm1__action__Act_SendGoal_Response__Sequence * input,
  srm1__action__Act_SendGoal_Response__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_GetResult_Request
 * )) before or use
 * srm1__action__Act_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Request__init(srm1__action__Act_GetResult_Request * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Request__fini(srm1__action__Act_GetResult_Request * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_GetResult_Request *
srm1__action__Act_GetResult_Request__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Request__destroy(srm1__action__Act_GetResult_Request * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Request__are_equal(const srm1__action__Act_GetResult_Request * lhs, const srm1__action__Act_GetResult_Request * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Request__copy(
  const srm1__action__Act_GetResult_Request * input,
  srm1__action__Act_GetResult_Request * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Request__Sequence__init(srm1__action__Act_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Request__Sequence__fini(srm1__action__Act_GetResult_Request__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_GetResult_Request__Sequence *
srm1__action__Act_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Request__Sequence__destroy(srm1__action__Act_GetResult_Request__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Request__Sequence__are_equal(const srm1__action__Act_GetResult_Request__Sequence * lhs, const srm1__action__Act_GetResult_Request__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Request__Sequence__copy(
  const srm1__action__Act_GetResult_Request__Sequence * input,
  srm1__action__Act_GetResult_Request__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_GetResult_Response
 * )) before or use
 * srm1__action__Act_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Response__init(srm1__action__Act_GetResult_Response * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Response__fini(srm1__action__Act_GetResult_Response * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_GetResult_Response *
srm1__action__Act_GetResult_Response__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Response__destroy(srm1__action__Act_GetResult_Response * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Response__are_equal(const srm1__action__Act_GetResult_Response * lhs, const srm1__action__Act_GetResult_Response * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Response__copy(
  const srm1__action__Act_GetResult_Response * input,
  srm1__action__Act_GetResult_Response * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Response__Sequence__init(srm1__action__Act_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Response__Sequence__fini(srm1__action__Act_GetResult_Response__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_GetResult_Response__Sequence *
srm1__action__Act_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_GetResult_Response__Sequence__destroy(srm1__action__Act_GetResult_Response__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Response__Sequence__are_equal(const srm1__action__Act_GetResult_Response__Sequence * lhs, const srm1__action__Act_GetResult_Response__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_GetResult_Response__Sequence__copy(
  const srm1__action__Act_GetResult_Response__Sequence * input,
  srm1__action__Act_GetResult_Response__Sequence * output);

/// Initialize action/Act message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * srm1__action__Act_FeedbackMessage
 * )) before or use
 * srm1__action__Act_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_FeedbackMessage__init(srm1__action__Act_FeedbackMessage * msg);

/// Finalize action/Act message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_FeedbackMessage__fini(srm1__action__Act_FeedbackMessage * msg);

/// Create action/Act message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * srm1__action__Act_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_FeedbackMessage *
srm1__action__Act_FeedbackMessage__create();

/// Destroy action/Act message.
/**
 * It calls
 * srm1__action__Act_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_FeedbackMessage__destroy(srm1__action__Act_FeedbackMessage * msg);

/// Check for action/Act message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_FeedbackMessage__are_equal(const srm1__action__Act_FeedbackMessage * lhs, const srm1__action__Act_FeedbackMessage * rhs);

/// Copy a action/Act message.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_FeedbackMessage__copy(
  const srm1__action__Act_FeedbackMessage * input,
  srm1__action__Act_FeedbackMessage * output);

/// Initialize array of action/Act messages.
/**
 * It allocates the memory for the number of elements and calls
 * srm1__action__Act_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_FeedbackMessage__Sequence__init(srm1__action__Act_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_FeedbackMessage__Sequence__fini(srm1__action__Act_FeedbackMessage__Sequence * array);

/// Create array of action/Act messages.
/**
 * It allocates the memory for the array and calls
 * srm1__action__Act_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
srm1__action__Act_FeedbackMessage__Sequence *
srm1__action__Act_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/Act messages.
/**
 * It calls
 * srm1__action__Act_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
void
srm1__action__Act_FeedbackMessage__Sequence__destroy(srm1__action__Act_FeedbackMessage__Sequence * array);

/// Check for action/Act message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_FeedbackMessage__Sequence__are_equal(const srm1__action__Act_FeedbackMessage__Sequence * lhs, const srm1__action__Act_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/Act messages.
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
ROSIDL_GENERATOR_C_PUBLIC_srm1
bool
srm1__action__Act_FeedbackMessage__Sequence__copy(
  const srm1__action__Act_FeedbackMessage__Sequence * input,
  srm1__action__Act_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SRM1__ACTION__DETAIL__ACT__FUNCTIONS_H_
