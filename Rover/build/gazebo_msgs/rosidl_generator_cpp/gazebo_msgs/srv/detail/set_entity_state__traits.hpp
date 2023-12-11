// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gazebo_msgs:srv/SetEntityState.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_MSGS__SRV__DETAIL__SET_ENTITY_STATE__TRAITS_HPP_
#define GAZEBO_MSGS__SRV__DETAIL__SET_ENTITY_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gazebo_msgs/srv/detail/set_entity_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'state'
#include "gazebo_msgs/msg/detail/entity_state__traits.hpp"

namespace gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetEntityState_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    to_flow_style_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetEntityState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state:\n";
    to_block_style_yaml(msg.state, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetEntityState_Request & msg, bool use_flow_style = false)
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

}  // namespace gazebo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use gazebo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gazebo_msgs::srv::SetEntityState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const gazebo_msgs::srv::SetEntityState_Request & msg)
{
  return gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gazebo_msgs::srv::SetEntityState_Request>()
{
  return "gazebo_msgs::srv::SetEntityState_Request";
}

template<>
inline const char * name<gazebo_msgs::srv::SetEntityState_Request>()
{
  return "gazebo_msgs/srv/SetEntityState_Request";
}

template<>
struct has_fixed_size<gazebo_msgs::srv::SetEntityState_Request>
  : std::integral_constant<bool, has_fixed_size<gazebo_msgs::msg::EntityState>::value> {};

template<>
struct has_bounded_size<gazebo_msgs::srv::SetEntityState_Request>
  : std::integral_constant<bool, has_bounded_size<gazebo_msgs::msg::EntityState>::value> {};

template<>
struct is_message<gazebo_msgs::srv::SetEntityState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetEntityState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetEntityState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetEntityState_Response & msg, bool use_flow_style = false)
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

}  // namespace gazebo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use gazebo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gazebo_msgs::srv::SetEntityState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const gazebo_msgs::srv::SetEntityState_Response & msg)
{
  return gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gazebo_msgs::srv::SetEntityState_Response>()
{
  return "gazebo_msgs::srv::SetEntityState_Response";
}

template<>
inline const char * name<gazebo_msgs::srv::SetEntityState_Response>()
{
  return "gazebo_msgs/srv/SetEntityState_Response";
}

template<>
struct has_fixed_size<gazebo_msgs::srv::SetEntityState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gazebo_msgs::srv::SetEntityState_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gazebo_msgs::srv::SetEntityState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace gazebo_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetEntityState_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetEntityState_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetEntityState_Event & msg, bool use_flow_style = false)
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

}  // namespace gazebo_msgs

namespace rosidl_generator_traits
{

[[deprecated("use gazebo_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gazebo_msgs::srv::SetEntityState_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  gazebo_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gazebo_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const gazebo_msgs::srv::SetEntityState_Event & msg)
{
  return gazebo_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gazebo_msgs::srv::SetEntityState_Event>()
{
  return "gazebo_msgs::srv::SetEntityState_Event";
}

template<>
inline const char * name<gazebo_msgs::srv::SetEntityState_Event>()
{
  return "gazebo_msgs/srv/SetEntityState_Event";
}

template<>
struct has_fixed_size<gazebo_msgs::srv::SetEntityState_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<gazebo_msgs::srv::SetEntityState_Event>
  : std::integral_constant<bool, has_bounded_size<gazebo_msgs::srv::SetEntityState_Request>::value && has_bounded_size<gazebo_msgs::srv::SetEntityState_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<gazebo_msgs::srv::SetEntityState_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gazebo_msgs::srv::SetEntityState>()
{
  return "gazebo_msgs::srv::SetEntityState";
}

template<>
inline const char * name<gazebo_msgs::srv::SetEntityState>()
{
  return "gazebo_msgs/srv/SetEntityState";
}

template<>
struct has_fixed_size<gazebo_msgs::srv::SetEntityState>
  : std::integral_constant<
    bool,
    has_fixed_size<gazebo_msgs::srv::SetEntityState_Request>::value &&
    has_fixed_size<gazebo_msgs::srv::SetEntityState_Response>::value
  >
{
};

template<>
struct has_bounded_size<gazebo_msgs::srv::SetEntityState>
  : std::integral_constant<
    bool,
    has_bounded_size<gazebo_msgs::srv::SetEntityState_Request>::value &&
    has_bounded_size<gazebo_msgs::srv::SetEntityState_Response>::value
  >
{
};

template<>
struct is_service<gazebo_msgs::srv::SetEntityState>
  : std::true_type
{
};

template<>
struct is_service_request<gazebo_msgs::srv::SetEntityState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gazebo_msgs::srv::SetEntityState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GAZEBO_MSGS__SRV__DETAIL__SET_ENTITY_STATE__TRAITS_HPP_
