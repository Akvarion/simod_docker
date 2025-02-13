// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from srm1:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef SRM1__MSG__DETAIL__MESSAGE__TRAITS_HPP_
#define SRM1__MSG__DETAIL__MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "srm1/msg/detail/message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace srm1
{

namespace msg
{

inline void to_flow_style_yaml(
  const Message & msg,
  std::ostream & out)
{
  out << "{";
  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Message & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Message & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace srm1

namespace rosidl_generator_traits
{

[[deprecated("use srm1::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const srm1::msg::Message & msg,
  std::ostream & out, size_t indentation = 0)
{
  srm1::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use srm1::msg::to_yaml() instead")]]
inline std::string to_yaml(const srm1::msg::Message & msg)
{
  return srm1::msg::to_yaml(msg);
}

template<>
inline const char * data_type<srm1::msg::Message>()
{
  return "srm1::msg::Message";
}

template<>
inline const char * name<srm1::msg::Message>()
{
  return "srm1/msg/Message";
}

template<>
struct has_fixed_size<srm1::msg::Message>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<srm1::msg::Message>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<srm1::msg::Message>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SRM1__MSG__DETAIL__MESSAGE__TRAITS_HPP_
