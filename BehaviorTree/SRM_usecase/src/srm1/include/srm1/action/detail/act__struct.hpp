// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from srm1:action/Act.idl
// generated code does not contain a copyright notice

#ifndef SRM1__ACTION__DETAIL__ACT__STRUCT_HPP_
#define SRM1__ACTION__DETAIL__ACT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_Goal __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_Goal __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_Goal_
{
  using Type = Act_Goal_<ContainerAllocator>;

  explicit Act_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  explicit Act_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->request = "";
    }
  }

  // field types and members
  using _request_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_Goal
    std::shared_ptr<srm1::action::Act_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_Goal
    std::shared_ptr<srm1::action::Act_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_Goal_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_Goal_

// alias to use template instance with default allocator
using Act_Goal =
  srm1::action::Act_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1


#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_Result __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_Result __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_Result_
{
  using Type = Act_Result_<ContainerAllocator>;

  explicit Act_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  explicit Act_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->response = "";
    }
  }

  // field types and members
  using _response_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_Result
    std::shared_ptr<srm1::action::Act_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_Result
    std::shared_ptr<srm1::action::Act_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_Result_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_Result_

// alias to use template instance with default allocator
using Act_Result =
  srm1::action::Act_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1


#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_Feedback __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_Feedback_
{
  using Type = Act_Feedback_<ContainerAllocator>;

  explicit Act_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback = "";
    }
  }

  explicit Act_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback = "";
    }
  }

  // field types and members
  using _feedback_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__feedback(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_Feedback
    std::shared_ptr<srm1::action::Act_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_Feedback
    std::shared_ptr<srm1::action::Act_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_Feedback_ & other) const
  {
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_Feedback_

// alias to use template instance with default allocator
using Act_Feedback =
  srm1::action::Act_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "srm1/action/detail/act__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_SendGoal_Request __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_SendGoal_Request_
{
  using Type = Act_SendGoal_Request_<ContainerAllocator>;

  explicit Act_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Act_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    srm1::action::Act_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const srm1::action::Act_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_SendGoal_Request
    std::shared_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_SendGoal_Request
    std::shared_ptr<srm1::action::Act_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_SendGoal_Request_

// alias to use template instance with default allocator
using Act_SendGoal_Request =
  srm1::action::Act_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_SendGoal_Response __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_SendGoal_Response_
{
  using Type = Act_SendGoal_Response_<ContainerAllocator>;

  explicit Act_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Act_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_SendGoal_Response
    std::shared_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_SendGoal_Response
    std::shared_ptr<srm1::action::Act_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_SendGoal_Response_

// alias to use template instance with default allocator
using Act_SendGoal_Response =
  srm1::action::Act_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1

namespace srm1
{

namespace action
{

struct Act_SendGoal
{
  using Request = srm1::action::Act_SendGoal_Request;
  using Response = srm1::action::Act_SendGoal_Response;
};

}  // namespace action

}  // namespace srm1


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_GetResult_Request __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_GetResult_Request_
{
  using Type = Act_GetResult_Request_<ContainerAllocator>;

  explicit Act_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Act_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_GetResult_Request
    std::shared_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_GetResult_Request
    std::shared_ptr<srm1::action::Act_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_GetResult_Request_

// alias to use template instance with default allocator
using Act_GetResult_Request =
  srm1::action::Act_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1


// Include directives for member types
// Member 'result'
// already included above
// #include "srm1/action/detail/act__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_GetResult_Response __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_GetResult_Response_
{
  using Type = Act_GetResult_Response_<ContainerAllocator>;

  explicit Act_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Act_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    srm1::action::Act_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const srm1::action::Act_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_GetResult_Response
    std::shared_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_GetResult_Response
    std::shared_ptr<srm1::action::Act_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_GetResult_Response_

// alias to use template instance with default allocator
using Act_GetResult_Response =
  srm1::action::Act_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1

namespace srm1
{

namespace action
{

struct Act_GetResult
{
  using Request = srm1::action::Act_GetResult_Request;
  using Response = srm1::action::Act_GetResult_Response;
};

}  // namespace action

}  // namespace srm1


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "srm1/action/detail/act__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__srm1__action__Act_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__srm1__action__Act_FeedbackMessage __declspec(deprecated)
#endif

namespace srm1
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Act_FeedbackMessage_
{
  using Type = Act_FeedbackMessage_<ContainerAllocator>;

  explicit Act_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Act_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    srm1::action::Act_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const srm1::action::Act_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::action::Act_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::action::Act_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::action::Act_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__action__Act_FeedbackMessage
    std::shared_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__action__Act_FeedbackMessage
    std::shared_ptr<srm1::action::Act_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Act_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Act_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Act_FeedbackMessage_

// alias to use template instance with default allocator
using Act_FeedbackMessage =
  srm1::action::Act_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace srm1

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace srm1
{

namespace action
{

struct Act
{
  /// The goal message defined in the action definition.
  using Goal = srm1::action::Act_Goal;
  /// The result message defined in the action definition.
  using Result = srm1::action::Act_Result;
  /// The feedback message defined in the action definition.
  using Feedback = srm1::action::Act_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = srm1::action::Act_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = srm1::action::Act_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = srm1::action::Act_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Act Act;

}  // namespace action

}  // namespace srm1

#endif  // SRM1__ACTION__DETAIL__ACT__STRUCT_HPP_
