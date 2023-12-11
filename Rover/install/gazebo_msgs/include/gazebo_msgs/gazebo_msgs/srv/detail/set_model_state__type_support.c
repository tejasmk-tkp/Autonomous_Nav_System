// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gazebo_msgs:srv/SetModelState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gazebo_msgs/srv/detail/set_model_state__rosidl_typesupport_introspection_c.h"
#include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gazebo_msgs/srv/detail/set_model_state__functions.h"
#include "gazebo_msgs/srv/detail/set_model_state__struct.h"


// Include directives for member types
// Member `model_state`
#include "gazebo_msgs/msg/model_state.h"
// Member `model_state`
#include "gazebo_msgs/msg/detail/model_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_msgs__srv__SetModelState_Request__init(message_memory);
}

void gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_fini_function(void * message_memory)
{
  gazebo_msgs__srv__SetModelState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_member_array[1] = {
  {
    "model_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelState_Request, model_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_members = {
  "gazebo_msgs__srv",  // message namespace
  "SetModelState_Request",  // message name
  1,  // number of fields
  sizeof(gazebo_msgs__srv__SetModelState_Request),
  gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_member_array,  // message members
  gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_type_support_handle = {
  0,
  &gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_members,
  get_message_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelState_Request__get_type_hash,
  &gazebo_msgs__srv__SetModelState_Request__get_type_description,
  &gazebo_msgs__srv__SetModelState_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Request)() {
  gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, msg, ModelState)();
  if (!gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__functions.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__struct.h"


// Include directives for member types
// Member `status_message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_msgs__srv__SetModelState_Response__init(message_memory);
}

void gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_fini_function(void * message_memory)
{
  gazebo_msgs__srv__SetModelState_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelState_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelState_Response, status_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_members = {
  "gazebo_msgs__srv",  // message namespace
  "SetModelState_Response",  // message name
  2,  // number of fields
  sizeof(gazebo_msgs__srv__SetModelState_Response),
  gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_member_array,  // message members
  gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle = {
  0,
  &gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_members,
  get_message_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelState_Response__get_type_hash,
  &gazebo_msgs__srv__SetModelState_Response__get_type_description,
  &gazebo_msgs__srv__SetModelState_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Response)() {
  if (!gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__functions.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "gazebo_msgs/srv/set_model_state.h"
// Member `request`
// Member `response`
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_msgs__srv__SetModelState_Event__init(message_memory);
}

void gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_fini_function(void * message_memory)
{
  gazebo_msgs__srv__SetModelState_Event__fini(message_memory);
}

size_t gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__size_function__SetModelState_Event__request(
  const void * untyped_member)
{
  const gazebo_msgs__srv__SetModelState_Request__Sequence * member =
    (const gazebo_msgs__srv__SetModelState_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelState_Event__request(
  const void * untyped_member, size_t index)
{
  const gazebo_msgs__srv__SetModelState_Request__Sequence * member =
    (const gazebo_msgs__srv__SetModelState_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_function__SetModelState_Event__request(
  void * untyped_member, size_t index)
{
  gazebo_msgs__srv__SetModelState_Request__Sequence * member =
    (gazebo_msgs__srv__SetModelState_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelState_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const gazebo_msgs__srv__SetModelState_Request * item =
    ((const gazebo_msgs__srv__SetModelState_Request *)
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelState_Event__request(untyped_member, index));
  gazebo_msgs__srv__SetModelState_Request * value =
    (gazebo_msgs__srv__SetModelState_Request *)(untyped_value);
  *value = *item;
}

void gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__assign_function__SetModelState_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  gazebo_msgs__srv__SetModelState_Request * item =
    ((gazebo_msgs__srv__SetModelState_Request *)
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_function__SetModelState_Event__request(untyped_member, index));
  const gazebo_msgs__srv__SetModelState_Request * value =
    (const gazebo_msgs__srv__SetModelState_Request *)(untyped_value);
  *item = *value;
}

bool gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__resize_function__SetModelState_Event__request(
  void * untyped_member, size_t size)
{
  gazebo_msgs__srv__SetModelState_Request__Sequence * member =
    (gazebo_msgs__srv__SetModelState_Request__Sequence *)(untyped_member);
  gazebo_msgs__srv__SetModelState_Request__Sequence__fini(member);
  return gazebo_msgs__srv__SetModelState_Request__Sequence__init(member, size);
}

size_t gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__size_function__SetModelState_Event__response(
  const void * untyped_member)
{
  const gazebo_msgs__srv__SetModelState_Response__Sequence * member =
    (const gazebo_msgs__srv__SetModelState_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelState_Event__response(
  const void * untyped_member, size_t index)
{
  const gazebo_msgs__srv__SetModelState_Response__Sequence * member =
    (const gazebo_msgs__srv__SetModelState_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_function__SetModelState_Event__response(
  void * untyped_member, size_t index)
{
  gazebo_msgs__srv__SetModelState_Response__Sequence * member =
    (gazebo_msgs__srv__SetModelState_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelState_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const gazebo_msgs__srv__SetModelState_Response * item =
    ((const gazebo_msgs__srv__SetModelState_Response *)
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelState_Event__response(untyped_member, index));
  gazebo_msgs__srv__SetModelState_Response * value =
    (gazebo_msgs__srv__SetModelState_Response *)(untyped_value);
  *value = *item;
}

void gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__assign_function__SetModelState_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  gazebo_msgs__srv__SetModelState_Response * item =
    ((gazebo_msgs__srv__SetModelState_Response *)
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_function__SetModelState_Event__response(untyped_member, index));
  const gazebo_msgs__srv__SetModelState_Response * value =
    (const gazebo_msgs__srv__SetModelState_Response *)(untyped_value);
  *item = *value;
}

bool gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__resize_function__SetModelState_Event__response(
  void * untyped_member, size_t size)
{
  gazebo_msgs__srv__SetModelState_Response__Sequence * member =
    (gazebo_msgs__srv__SetModelState_Response__Sequence *)(untyped_member);
  gazebo_msgs__srv__SetModelState_Response__Sequence__fini(member);
  return gazebo_msgs__srv__SetModelState_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelState_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelState_Event, request),  // bytes offset in struct
    NULL,  // default value
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__size_function__SetModelState_Event__request,  // size() function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelState_Event__request,  // get_const(index) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_function__SetModelState_Event__request,  // get(index) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelState_Event__request,  // fetch(index, &value) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__assign_function__SetModelState_Event__request,  // assign(index, value) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__resize_function__SetModelState_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelState_Event, response),  // bytes offset in struct
    NULL,  // default value
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__size_function__SetModelState_Event__response,  // size() function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelState_Event__response,  // get_const(index) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__get_function__SetModelState_Event__response,  // get(index) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelState_Event__response,  // fetch(index, &value) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__assign_function__SetModelState_Event__response,  // assign(index, value) function pointer
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__resize_function__SetModelState_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_members = {
  "gazebo_msgs__srv",  // message namespace
  "SetModelState_Event",  // message name
  3,  // number of fields
  sizeof(gazebo_msgs__srv__SetModelState_Event),
  gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_member_array,  // message members
  gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_type_support_handle = {
  0,
  &gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_members,
  get_message_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelState_Event__get_type_hash,
  &gazebo_msgs__srv__SetModelState_Event__get_type_description,
  &gazebo_msgs__srv__SetModelState_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Event)() {
  gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Request)();
  gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Response)();
  if (!gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_members = {
  "gazebo_msgs__srv",  // service namespace
  "SetModelState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_Request_message_type_support_handle,
  NULL,  // response message
  // gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle
  NULL  // event_message
  // gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle
};


static rosidl_service_type_support_t gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_type_support_handle = {
  0,
  &gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_members,
  get_service_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelState_Request__rosidl_typesupport_introspection_c__SetModelState_Request_message_type_support_handle,
  &gazebo_msgs__srv__SetModelState_Response__rosidl_typesupport_introspection_c__SetModelState_Response_message_type_support_handle,
  &gazebo_msgs__srv__SetModelState_Event__rosidl_typesupport_introspection_c__SetModelState_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    gazebo_msgs,
    srv,
    SetModelState
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    gazebo_msgs,
    srv,
    SetModelState
  ),
  &gazebo_msgs__srv__SetModelState__get_type_hash,
  &gazebo_msgs__srv__SetModelState__get_type_description,
  &gazebo_msgs__srv__SetModelState__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState)() {
  if (!gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelState_Event)()->data;
  }

  return &gazebo_msgs__srv__detail__set_model_state__rosidl_typesupport_introspection_c__SetModelState_service_type_support_handle;
}
