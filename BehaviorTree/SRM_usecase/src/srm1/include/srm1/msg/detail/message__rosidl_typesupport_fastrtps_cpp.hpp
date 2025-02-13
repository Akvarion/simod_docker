// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from srm1:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef SRM1__MSG__DETAIL__MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SRM1__MSG__DETAIL__MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "srm1/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "srm1/msg/detail/message__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace srm1
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_srm1
cdr_serialize(
  const srm1::msg::Message & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_srm1
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  srm1::msg::Message & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_srm1
get_serialized_size(
  const srm1::msg::Message & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_srm1
max_serialized_size_Message(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace srm1

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_srm1
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, srm1, msg, Message)();

#ifdef __cplusplus
}
#endif

#endif  // SRM1__MSG__DETAIL__MESSAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
