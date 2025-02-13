// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from srm1:msg/Message.idl
// generated code does not contain a copyright notice

#ifndef SRM1__MSG__DETAIL__MESSAGE__BUILDER_HPP_
#define SRM1__MSG__DETAIL__MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "srm1/msg/detail/message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace srm1
{

namespace msg
{

namespace builder
{

class Init_Message_message
{
public:
  Init_Message_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::srm1::msg::Message message(::srm1::msg::Message::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::msg::Message msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::msg::Message>()
{
  return srm1::msg::builder::Init_Message_message();
}

}  // namespace srm1

#endif  // SRM1__MSG__DETAIL__MESSAGE__BUILDER_HPP_
