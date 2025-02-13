// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from srm1:srv/Service.idl
// generated code does not contain a copyright notice

#ifndef SRM1__SRV__DETAIL__SERVICE__STRUCT_HPP_
#define SRM1__SRV__DETAIL__SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__srm1__srv__Service_Request __attribute__((deprecated))
#else
# define DEPRECATED__srm1__srv__Service_Request __declspec(deprecated)
#endif

namespace srm1
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Service_Request_
{
  using Type = Service_Request_<ContainerAllocator>;

  explicit Service_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->coordinates.begin(), this->coordinates.end(), 0.0f);
      this->distance = 0.0f;
    }
  }

  explicit Service_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : coordinates(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->coordinates.begin(), this->coordinates.end(), 0.0f);
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _coordinates_type =
    std::array<float, 4>;
  _coordinates_type coordinates;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__coordinates(
    const std::array<float, 4> & _arg)
  {
    this->coordinates = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    srm1::srv::Service_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::srv::Service_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::srv::Service_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::srv::Service_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::srv::Service_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::srv::Service_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::srv::Service_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::srv::Service_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::srv::Service_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::srv::Service_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__srv__Service_Request
    std::shared_ptr<srm1::srv::Service_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__srv__Service_Request
    std::shared_ptr<srm1::srv::Service_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Service_Request_ & other) const
  {
    if (this->coordinates != other.coordinates) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Service_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Service_Request_

// alias to use template instance with default allocator
using Service_Request =
  srm1::srv::Service_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace srm1


#ifndef _WIN32
# define DEPRECATED__srm1__srv__Service_Response __attribute__((deprecated))
#else
# define DEPRECATED__srm1__srv__Service_Response __declspec(deprecated)
#endif

namespace srm1
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Service_Response_
{
  using Type = Service_Response_<ContainerAllocator>;

  explicit Service_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Service_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    srm1::srv::Service_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const srm1::srv::Service_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<srm1::srv::Service_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<srm1::srv::Service_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      srm1::srv::Service_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<srm1::srv::Service_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      srm1::srv::Service_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<srm1::srv::Service_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<srm1::srv::Service_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<srm1::srv::Service_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__srm1__srv__Service_Response
    std::shared_ptr<srm1::srv::Service_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__srm1__srv__Service_Response
    std::shared_ptr<srm1::srv::Service_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Service_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Service_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Service_Response_

// alias to use template instance with default allocator
using Service_Response =
  srm1::srv::Service_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace srm1

namespace srm1
{

namespace srv
{

struct Service
{
  using Request = srm1::srv::Service_Request;
  using Response = srm1::srv::Service_Response;
};

}  // namespace srv

}  // namespace srm1

#endif  // SRM1__SRV__DETAIL__SERVICE__STRUCT_HPP_
