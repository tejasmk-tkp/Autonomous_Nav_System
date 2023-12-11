// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gazebo_msgs:srv/SetModelConfiguration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gazebo_msgs/srv/detail/set_model_configuration__rosidl_typesupport_introspection_c.h"
#include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gazebo_msgs/srv/detail/set_model_configuration__functions.h"
#include "gazebo_msgs/srv/detail/set_model_configuration__struct.h"


// Include directives for member types
// Member `model_name`
// Member `urdf_param_name`
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `joint_positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_msgs__srv__SetModelConfiguration_Request__init(message_memory);
}

void gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_fini_function(void * message_memory)
{
  gazebo_msgs__srv__SetModelConfiguration_Request__fini(message_memory);
}

size_t gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Request__joint_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Request__joint_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Request__joint_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Request__joint_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Request__joint_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Request__joint_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Request__joint_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Request__joint_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Request__joint_positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Request__joint_positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Request__joint_positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Request__joint_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Request__joint_positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Request__joint_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Request__joint_positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Request__joint_positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_member_array[4] = {
  {
    "model_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Request, model_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "urdf_param_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Request, urdf_param_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Request, joint_names),  // bytes offset in struct
    NULL,  // default value
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Request__joint_names,  // size() function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Request__joint_names,  // get_const(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Request__joint_names,  // get(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Request__joint_names,  // fetch(index, &value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Request__joint_names,  // assign(index, value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Request__joint_names  // resize(index) function pointer
  },
  {
    "joint_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Request, joint_positions),  // bytes offset in struct
    NULL,  // default value
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Request__joint_positions,  // size() function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Request__joint_positions,  // get_const(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Request__joint_positions,  // get(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Request__joint_positions,  // fetch(index, &value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Request__joint_positions,  // assign(index, value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Request__joint_positions  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_members = {
  "gazebo_msgs__srv",  // message namespace
  "SetModelConfiguration_Request",  // message name
  4,  // number of fields
  sizeof(gazebo_msgs__srv__SetModelConfiguration_Request),
  gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_member_array,  // message members
  gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_type_support_handle = {
  0,
  &gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_members,
  get_message_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelConfiguration_Request__get_type_hash,
  &gazebo_msgs__srv__SetModelConfiguration_Request__get_type_description,
  &gazebo_msgs__srv__SetModelConfiguration_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Request)() {
  if (!gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__functions.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__struct.h"


// Include directives for member types
// Member `status_message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_msgs__srv__SetModelConfiguration_Response__init(message_memory);
}

void gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_fini_function(void * message_memory)
{
  gazebo_msgs__srv__SetModelConfiguration_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Response, success),  // bytes offset in struct
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
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Response, status_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_members = {
  "gazebo_msgs__srv",  // message namespace
  "SetModelConfiguration_Response",  // message name
  2,  // number of fields
  sizeof(gazebo_msgs__srv__SetModelConfiguration_Response),
  gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_member_array,  // message members
  gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle = {
  0,
  &gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_members,
  get_message_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelConfiguration_Response__get_type_hash,
  &gazebo_msgs__srv__SetModelConfiguration_Response__get_type_description,
  &gazebo_msgs__srv__SetModelConfiguration_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Response)() {
  if (!gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__functions.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "gazebo_msgs/srv/set_model_configuration.h"
// Member `request`
// Member `response`
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gazebo_msgs__srv__SetModelConfiguration_Event__init(message_memory);
}

void gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_fini_function(void * message_memory)
{
  gazebo_msgs__srv__SetModelConfiguration_Event__fini(message_memory);
}

size_t gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Event__request(
  const void * untyped_member)
{
  const gazebo_msgs__srv__SetModelConfiguration_Request__Sequence * member =
    (const gazebo_msgs__srv__SetModelConfiguration_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Event__request(
  const void * untyped_member, size_t index)
{
  const gazebo_msgs__srv__SetModelConfiguration_Request__Sequence * member =
    (const gazebo_msgs__srv__SetModelConfiguration_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Event__request(
  void * untyped_member, size_t index)
{
  gazebo_msgs__srv__SetModelConfiguration_Request__Sequence * member =
    (gazebo_msgs__srv__SetModelConfiguration_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const gazebo_msgs__srv__SetModelConfiguration_Request * item =
    ((const gazebo_msgs__srv__SetModelConfiguration_Request *)
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Event__request(untyped_member, index));
  gazebo_msgs__srv__SetModelConfiguration_Request * value =
    (gazebo_msgs__srv__SetModelConfiguration_Request *)(untyped_value);
  *value = *item;
}

void gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  gazebo_msgs__srv__SetModelConfiguration_Request * item =
    ((gazebo_msgs__srv__SetModelConfiguration_Request *)
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Event__request(untyped_member, index));
  const gazebo_msgs__srv__SetModelConfiguration_Request * value =
    (const gazebo_msgs__srv__SetModelConfiguration_Request *)(untyped_value);
  *item = *value;
}

bool gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Event__request(
  void * untyped_member, size_t size)
{
  gazebo_msgs__srv__SetModelConfiguration_Request__Sequence * member =
    (gazebo_msgs__srv__SetModelConfiguration_Request__Sequence *)(untyped_member);
  gazebo_msgs__srv__SetModelConfiguration_Request__Sequence__fini(member);
  return gazebo_msgs__srv__SetModelConfiguration_Request__Sequence__init(member, size);
}

size_t gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Event__response(
  const void * untyped_member)
{
  const gazebo_msgs__srv__SetModelConfiguration_Response__Sequence * member =
    (const gazebo_msgs__srv__SetModelConfiguration_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Event__response(
  const void * untyped_member, size_t index)
{
  const gazebo_msgs__srv__SetModelConfiguration_Response__Sequence * member =
    (const gazebo_msgs__srv__SetModelConfiguration_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Event__response(
  void * untyped_member, size_t index)
{
  gazebo_msgs__srv__SetModelConfiguration_Response__Sequence * member =
    (gazebo_msgs__srv__SetModelConfiguration_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const gazebo_msgs__srv__SetModelConfiguration_Response * item =
    ((const gazebo_msgs__srv__SetModelConfiguration_Response *)
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Event__response(untyped_member, index));
  gazebo_msgs__srv__SetModelConfiguration_Response * value =
    (gazebo_msgs__srv__SetModelConfiguration_Response *)(untyped_value);
  *value = *item;
}

void gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  gazebo_msgs__srv__SetModelConfiguration_Response * item =
    ((gazebo_msgs__srv__SetModelConfiguration_Response *)
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Event__response(untyped_member, index));
  const gazebo_msgs__srv__SetModelConfiguration_Response * value =
    (const gazebo_msgs__srv__SetModelConfiguration_Response *)(untyped_value);
  *item = *value;
}

bool gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Event__response(
  void * untyped_member, size_t size)
{
  gazebo_msgs__srv__SetModelConfiguration_Response__Sequence * member =
    (gazebo_msgs__srv__SetModelConfiguration_Response__Sequence *)(untyped_member);
  gazebo_msgs__srv__SetModelConfiguration_Response__Sequence__fini(member);
  return gazebo_msgs__srv__SetModelConfiguration_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Event, info),  // bytes offset in struct
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
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Event, request),  // bytes offset in struct
    NULL,  // default value
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Event__request,  // size() function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Event__request,  // get_const(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Event__request,  // get(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Event__request,  // fetch(index, &value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Event__request,  // assign(index, value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(gazebo_msgs__srv__SetModelConfiguration_Event, response),  // bytes offset in struct
    NULL,  // default value
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__size_function__SetModelConfiguration_Event__response,  // size() function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_const_function__SetModelConfiguration_Event__response,  // get_const(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__get_function__SetModelConfiguration_Event__response,  // get(index) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__fetch_function__SetModelConfiguration_Event__response,  // fetch(index, &value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__assign_function__SetModelConfiguration_Event__response,  // assign(index, value) function pointer
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__resize_function__SetModelConfiguration_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_members = {
  "gazebo_msgs__srv",  // message namespace
  "SetModelConfiguration_Event",  // message name
  3,  // number of fields
  sizeof(gazebo_msgs__srv__SetModelConfiguration_Event),
  gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_member_array,  // message members
  gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_type_support_handle = {
  0,
  &gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_members,
  get_message_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelConfiguration_Event__get_type_hash,
  &gazebo_msgs__srv__SetModelConfiguration_Event__get_type_description,
  &gazebo_msgs__srv__SetModelConfiguration_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Event)() {
  gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Request)();
  gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Response)();
  if (!gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "gazebo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "gazebo_msgs/srv/detail/set_model_configuration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_members = {
  "gazebo_msgs__srv",  // service namespace
  "SetModelConfiguration",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_type_support_handle,
  NULL,  // response message
  // gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle
  NULL  // event_message
  // gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle
};


static rosidl_service_type_support_t gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_type_support_handle = {
  0,
  &gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_members,
  get_service_typesupport_handle_function,
  &gazebo_msgs__srv__SetModelConfiguration_Request__rosidl_typesupport_introspection_c__SetModelConfiguration_Request_message_type_support_handle,
  &gazebo_msgs__srv__SetModelConfiguration_Response__rosidl_typesupport_introspection_c__SetModelConfiguration_Response_message_type_support_handle,
  &gazebo_msgs__srv__SetModelConfiguration_Event__rosidl_typesupport_introspection_c__SetModelConfiguration_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    gazebo_msgs,
    srv,
    SetModelConfiguration
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    gazebo_msgs,
    srv,
    SetModelConfiguration
  ),
  &gazebo_msgs__srv__SetModelConfiguration__get_type_hash,
  &gazebo_msgs__srv__SetModelConfiguration__get_type_description,
  &gazebo_msgs__srv__SetModelConfiguration__get_type_description_sources,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Response)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Event)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gazebo_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration)() {
  if (!gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_type_support_handle.typesupport_identifier) {
    gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gazebo_msgs, srv, SetModelConfiguration_Event)()->data;
  }

  return &gazebo_msgs__srv__detail__set_model_configuration__rosidl_typesupport_introspection_c__SetModelConfiguration_service_type_support_handle;
}
