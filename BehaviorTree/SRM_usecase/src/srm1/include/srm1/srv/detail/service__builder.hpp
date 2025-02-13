// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from srm1:srv/Service.idl
// generated code does not contain a copyright notice

#ifndef SRM1__SRV__DETAIL__SERVICE__BUILDER_HPP_
#define SRM1__SRV__DETAIL__SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "srm1/srv/detail/service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace srm1
{

namespace srv
{

namespace builder
{

class Init_Service_Request_distance
{
public:
  explicit Init_Service_Request_distance(::srm1::srv::Service_Request & msg)
  : msg_(msg)
  {}
  ::srm1::srv::Service_Request distance(::srm1::srv::Service_Request::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::srv::Service_Request msg_;
};

class Init_Service_Request_coordinates
{
public:
  Init_Service_Request_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Service_Request_distance coordinates(::srm1::srv::Service_Request::_coordinates_type arg)
  {
    msg_.coordinates = std::move(arg);
    return Init_Service_Request_distance(msg_);
  }

private:
  ::srm1::srv::Service_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::srv::Service_Request>()
{
  return srm1::srv::builder::Init_Service_Request_coordinates();
}

}  // namespace srm1


namespace srm1
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::srv::Service_Response>()
{
  return ::srm1::srv::Service_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace srm1

#endif  // SRM1__SRV__DETAIL__SERVICE__BUILDER_HPP_
