// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gazebo_msgs:srv/SetLinkState.idl
// generated code does not contain a copyright notice

#ifndef GAZEBO_MSGS__SRV__DETAIL__SET_LINK_STATE__STRUCT_H_
#define GAZEBO_MSGS__SRV__DETAIL__SET_LINK_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'link_state'
#include "gazebo_msgs/msg/detail/link_state__struct.h"

/// Struct defined in srv/SetLinkState in the package gazebo_msgs.
typedef struct gazebo_msgs__srv__SetLinkState_Request
{
  gazebo_msgs__msg__LinkState link_state;
} gazebo_msgs__srv__SetLinkState_Request;

// Struct for a sequence of gazebo_msgs__srv__SetLinkState_Request.
typedef struct gazebo_msgs__srv__SetLinkState_Request__Sequence
{
  gazebo_msgs__srv__SetLinkState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gazebo_msgs__srv__SetLinkState_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'status_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetLinkState in the package gazebo_msgs.
typedef struct gazebo_msgs__srv__SetLinkState_Response
{
  /// return true if set wrench successful
  bool success;
  /// comments if available
  rosidl_runtime_c__String status_message;
} gazebo_msgs__srv__SetLinkState_Response;

// Struct for a sequence of gazebo_msgs__srv__SetLinkState_Response.
typedef struct gazebo_msgs__srv__SetLinkState_Response__Sequence
{
  gazebo_msgs__srv__SetLinkState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gazebo_msgs__srv__SetLinkState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  gazebo_msgs__srv__SetLinkState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  gazebo_msgs__srv__SetLinkState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetLinkState in the package gazebo_msgs.
typedef struct gazebo_msgs__srv__SetLinkState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  gazebo_msgs__srv__SetLinkState_Request__Sequence request;
  gazebo_msgs__srv__SetLinkState_Response__Sequence response;
} gazebo_msgs__srv__SetLinkState_Event;

// Struct for a sequence of gazebo_msgs__srv__SetLinkState_Event.
typedef struct gazebo_msgs__srv__SetLinkState_Event__Sequence
{
  gazebo_msgs__srv__SetLinkState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gazebo_msgs__srv__SetLinkState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GAZEBO_MSGS__SRV__DETAIL__SET_LINK_STATE__STRUCT_H_
