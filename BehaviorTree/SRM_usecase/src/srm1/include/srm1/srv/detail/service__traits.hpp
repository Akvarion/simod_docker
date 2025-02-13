// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from srm1:srv/Service.idl
// generated code does not contain a copyright notice

#ifndef SRM1__SRV__DETAIL__SERVICE__TRAITS_HPP_
#define SRM1__SRV__DETAIL__SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "srm1/srv/detail/service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace srm1
{

namespace srv
{

inline void to_flow_style_yaml(
  const Service_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: coordinates
  {
    if (msg.coordinates.size() == 0) {
      out << "coordinates: []";
    } else {
      out << "coordinates: [";
      size_t pending_items = msg.coordinates.size();
      for (auto item : msg.coordinates) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Service_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: coordinates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coordinates.size() == 0) {
      out << "coordinates: []\n";
    } else {
      out << "coordinates:\n";
      for (auto item : msg.coordinates) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Service_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace srm1

namespace rosidl_generator_traits
{

[[deprecated("use srm1::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const srm1::srv::Service_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  srm1::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use srm1::srv::to_yaml() instead")]]
inline std::string to_yaml(const srm1::srv::Service_Request & msg)
{
  return srm1::srv::to_yaml(msg);
}

template<>
inline const char * data_type<srm1::srv::Service_Request>()
{
  return "srm1::srv::Service_Request";
}

template<>
inline const char * name<srm1::srv::Service_Request>()
{
  return "srm1/srv/Service_Request";
}

template<>
struct has_fixed_size<srm1::srv::Service_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<srm1::srv::Service_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<srm1::srv::Service_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace srm1
{

namespace srv
{

inline void to_flow_style_yaml(
  const Service_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Service_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Service_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace srm1

namespace rosidl_generator_traits
{

[[deprecated("use srm1::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const srm1::srv::Service_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  srm1::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use srm1::srv::to_yaml() instead")]]
inline std::string to_yaml(const srm1::srv::Service_Response & msg)
{
  return srm1::srv::to_yaml(msg);
}

template<>
inline const char * data_type<srm1::srv::Service_Response>()
{
  return "srm1::srv::Service_Response";
}

template<>
inline const char * name<srm1::srv::Service_Response>()
{
  return "srm1/srv/Service_Response";
}

template<>
struct has_fixed_size<srm1::srv::Service_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<srm1::srv::Service_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<srm1::srv::Service_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<srm1::srv::Service>()
{
  return "srm1::srv::Service";
}

template<>
inline const char * name<srm1::srv::Service>()
{
  return "srm1/srv/Service";
}

template<>
struct has_fixed_size<srm1::srv::Service>
  : std::integral_constant<
    bool,
    has_fixed_size<srm1::srv::Service_Request>::value &&
    has_fixed_size<srm1::srv::Service_Response>::value
  >
{
};

template<>
struct has_bounded_size<srm1::srv::Service>
  : std::integral_constant<
    bool,
    has_bounded_size<srm1::srv::Service_Request>::value &&
    has_bounded_size<srm1::srv::Service_Response>::value
  >
{
};

template<>
struct is_service<srm1::srv::Service>
  : std::true_type
{
};

template<>
struct is_service_request<srm1::srv::Service_Request>
  : std::true_type
{
};

template<>
struct is_service_response<srm1::srv::Service_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SRM1__SRV__DETAIL__SERVICE__TRAITS_HPP_
