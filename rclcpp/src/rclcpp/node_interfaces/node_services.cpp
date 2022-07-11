// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/node_interfaces/node_services.hpp"
#include <rcl/service.h>
#include <rcl/types.h>
#include "rcl/introspection.h"

#include <string>
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter_value.hpp"

using rclcpp::node_interfaces::NodeServices;

NodeServices::NodeServices(rclcpp::node_interfaces::NodeBaseInterface * node_base)
: node_base_(node_base)
{}

NodeServices::~NodeServices()
{}

void
NodeServices::add_service(
  rclcpp::ServiceBase::SharedPtr service_base_ptr,
  rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create service, group not in node.");
    }
    group->add_service(service_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_service(service_base_ptr);
  }

  // Notify the executor that a new service was created using the parent Node.
  auto & node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("failed to notify wait set on service creation: ") + ex.what());
  }


  // Declare service introspection configuration parameters
  // TODO(ihasdapie): Should we register both the service and client ones together?
  static constexpr const char * publish_service_events_param_name = "publish_service_events";
  static constexpr const char * service_event_content_param_name = "service_event_content";

  // in retrospect do we even need to keep the parameter value after declaring?
  rclcpp::ParameterValue publish_service_events_param;
  rclcpp::ParameterValue service_event_content_param;


  if (!node_parameters_->has_parameter(publish_service_events_param_name)) {
    publish_service_events_param = node_parameters_->declare_parameter(
        publish_service_events_param_name, rclcpp::ParameterValue(true));
  } else {
    publish_service_events_param = node_parameters_->get_parameter(publish_service_events_param_name).get_parameter_value();
  }

  if (!node_parameters_->has_parameter(service_event_content_param_name)) {
    service_event_content_param = node_parameters_->declare_parameter(
        service_event_content_param_name, rclcpp::ParameterValue(true));
  } else {
    service_event_content_param = node_parameters_->get_parameter(service_event_content_param_name).get_parameter_value();
  }

  // Call corresponding enable/disable service introspection rcl functions on relevant parameter events
  auto configure_service_introspection_callback = 
    [this, service_base_ptr](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_node_t * node_handle = this->node_base_->get_rcl_node_handle();
      rcl_service_t * service_handle = service_base_ptr->get_service_handle().get();
      rcl_ret_t ret;

      for (const auto & param: parameters) {
        if (param.get_name() == publish_service_events_param_name) {
          ret = rcl_service_introspection_configure_service_events(service_handle, node_handle, param.get_value<bool>());
          if (RCL_RET_OK != ret) {
            throw std::runtime_error("Could not configure service introspection events");
          }
        } else if (param.get_name() == service_event_content_param_name) {
          rcl_service_introspection_configure_service_content(service_handle, param.get_value<bool>());
          if (RCL_RET_OK != ret) {
            throw std::runtime_error("Could not configure service introspection events");
          }
        }
      }
    };

  node_parameters_->add_post_set_parameters_callback(configure_service_introspection_callback);

}

void
NodeServices::add_client(
  rclcpp::ClientBase::SharedPtr client_base_ptr,
  rclcpp::CallbackGroup::SharedPtr group,
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_)
{
  if (group) {
    if (!node_base_->callback_group_in_node(group)) {
      // TODO(jacquelinekay): use custom exception
      throw std::runtime_error("Cannot create client, group not in node.");
    }
    group->add_client(client_base_ptr);
  } else {
    node_base_->get_default_callback_group()->add_client(client_base_ptr);
  }

  // Notify the executor that a new client was created using the parent Node.
  auto & node_gc = node_base_->get_notify_guard_condition();
  try {
    node_gc.trigger();
  } catch (const rclcpp::exceptions::RCLError & ex) {
    throw std::runtime_error(
            std::string("failed to notify wait set on client creation: ") + ex.what());
  }



  static constexpr const char * client_event_content_param_name = "client_event_content";
  static constexpr const char * publish_client_events_param_name = "publish_client_events";
  rclcpp::ParameterValue publish_client_event_param;
  rclcpp::ParameterValue client_event_content_param;

  if (!node_parameters_->has_parameter(publish_client_events_param_name)) {
    publish_client_event_param = publish_client_event_param = node_parameters_->declare_parameter(
        publish_client_events_param_name, rclcpp::ParameterValue(true));
  } else {
    publish_client_event_param = publish_client_event_param = node_parameters_->get_parameter(publish_client_events_param_name).get_parameter_value();
  }

  if (!node_parameters_->has_parameter(client_event_content_param_name)) {
    client_event_content_param = node_parameters_->declare_parameter(
        client_event_content_param_name, rclcpp::ParameterValue(true));
  } else {
    client_event_content_param = node_parameters_->get_parameter(client_event_content_param_name).get_parameter_value();
  }

  auto configure_service_introspection_callback = 
    [this, client_base_ptr](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_node_t * node_handle = this->node_base_->get_rcl_node_handle();
      rcl_client_t * client_handle = client_base_ptr->get_client_handle().get();
      rcl_ret_t ret;

      for (const auto & param: parameters) {
        if (param.get_name() == publish_client_events_param_name) {
          ret = rcl_service_introspection_configure_client_events(client_handle, node_handle, param.get_value<bool>());
          if (RCL_RET_OK != ret) {
            throw std::runtime_error("Could not configure service introspection events");
          }
        } else if (param.get_name() == client_event_content_param_name) {
          rcl_service_introspection_configure_client_content(client_handle, param.get_value<bool>());
          if (RCL_RET_OK != ret) {
            throw std::runtime_error("Could not configure service introspection events");
          }
        }
      }
    };

  node_parameters_->add_post_set_parameters_callback(configure_service_introspection_callback);




}

std::string
NodeServices::resolve_service_name(const std::string & name, bool only_expand) const
{
  return node_base_->resolve_topic_or_service_name(name, true, only_expand);
}
