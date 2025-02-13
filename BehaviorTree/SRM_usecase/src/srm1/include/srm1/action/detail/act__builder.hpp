// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from srm1:action/Act.idl
// generated code does not contain a copyright notice

#ifndef SRM1__ACTION__DETAIL__ACT__BUILDER_HPP_
#define SRM1__ACTION__DETAIL__ACT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "srm1/action/detail/act__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_Goal_request
{
public:
  Init_Act_Goal_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::srm1::action::Act_Goal request(::srm1::action::Act_Goal::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_Goal>()
{
  return srm1::action::builder::Init_Act_Goal_request();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_Result_response
{
public:
  Init_Act_Result_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::srm1::action::Act_Result response(::srm1::action::Act_Result::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_Result>()
{
  return srm1::action::builder::Init_Act_Result_response();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_Feedback_feedback
{
public:
  Init_Act_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::srm1::action::Act_Feedback feedback(::srm1::action::Act_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_Feedback>()
{
  return srm1::action::builder::Init_Act_Feedback_feedback();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_SendGoal_Request_goal
{
public:
  explicit Init_Act_SendGoal_Request_goal(::srm1::action::Act_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::srm1::action::Act_SendGoal_Request goal(::srm1::action::Act_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_SendGoal_Request msg_;
};

class Init_Act_SendGoal_Request_goal_id
{
public:
  Init_Act_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Act_SendGoal_Request_goal goal_id(::srm1::action::Act_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Act_SendGoal_Request_goal(msg_);
  }

private:
  ::srm1::action::Act_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_SendGoal_Request>()
{
  return srm1::action::builder::Init_Act_SendGoal_Request_goal_id();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_SendGoal_Response_stamp
{
public:
  explicit Init_Act_SendGoal_Response_stamp(::srm1::action::Act_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::srm1::action::Act_SendGoal_Response stamp(::srm1::action::Act_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_SendGoal_Response msg_;
};

class Init_Act_SendGoal_Response_accepted
{
public:
  Init_Act_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Act_SendGoal_Response_stamp accepted(::srm1::action::Act_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Act_SendGoal_Response_stamp(msg_);
  }

private:
  ::srm1::action::Act_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_SendGoal_Response>()
{
  return srm1::action::builder::Init_Act_SendGoal_Response_accepted();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_GetResult_Request_goal_id
{
public:
  Init_Act_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::srm1::action::Act_GetResult_Request goal_id(::srm1::action::Act_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_GetResult_Request>()
{
  return srm1::action::builder::Init_Act_GetResult_Request_goal_id();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_GetResult_Response_result
{
public:
  explicit Init_Act_GetResult_Response_result(::srm1::action::Act_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::srm1::action::Act_GetResult_Response result(::srm1::action::Act_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_GetResult_Response msg_;
};

class Init_Act_GetResult_Response_status
{
public:
  Init_Act_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Act_GetResult_Response_result status(::srm1::action::Act_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Act_GetResult_Response_result(msg_);
  }

private:
  ::srm1::action::Act_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_GetResult_Response>()
{
  return srm1::action::builder::Init_Act_GetResult_Response_status();
}

}  // namespace srm1


namespace srm1
{

namespace action
{

namespace builder
{

class Init_Act_FeedbackMessage_feedback
{
public:
  explicit Init_Act_FeedbackMessage_feedback(::srm1::action::Act_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::srm1::action::Act_FeedbackMessage feedback(::srm1::action::Act_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::srm1::action::Act_FeedbackMessage msg_;
};

class Init_Act_FeedbackMessage_goal_id
{
public:
  Init_Act_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Act_FeedbackMessage_feedback goal_id(::srm1::action::Act_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Act_FeedbackMessage_feedback(msg_);
  }

private:
  ::srm1::action::Act_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::srm1::action::Act_FeedbackMessage>()
{
  return srm1::action::builder::Init_Act_FeedbackMessage_goal_id();
}

}  // namespace srm1

#endif  // SRM1__ACTION__DETAIL__ACT__BUILDER_HPP_
