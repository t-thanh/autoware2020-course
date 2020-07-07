// Copyright [2020] [Daniel Peter, peter@fh-aachen.de, Fachhochschule Aachen]
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.


#ifndef COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_
#define COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <map>

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <can_msgs/msg/frame.hpp>
#include <radar_conti_ars408_msgs/msg/object_list.hpp>
#include <radar_conti_ars408_msgs/msg/cluster_list.hpp>
#include <radar_conti_ars408_msgs/msg/radar_state.hpp>
#include <radar_conti_ars408_msgs/msg/cluster_status.hpp>


#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/subscription_factory.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/timer.hpp"
#include "rmw/qos_profiles.h"

#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include "ros2socketcan_bridge/ros2socketcan.h"
#include <ars_408_can_defines.h>

constexpr char DEFAULT_NODE_NAME[] = "RADAR_CONTI_ARS408";


typedef unsigned char ubyte;
typedef unsigned short int uword;

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;


namespace FHAC
{

class radar_conti_ars408 : public rclcpp_lifecycle::LifecycleNode
{
public:
  RADAR_CONTI_ARS408_PUBLIC
  radar_conti_ars408(const rclcpp::NodeOptions & options);

        /// Transition callback for state error
        /**
        * on_error callback is being called when the lifecycle node
        * enters the "error" state.
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
            const rclcpp_lifecycle::State&
                previous_state);

        /// Transition callback for state shutting down
        /**
        * on_shutdown callback is being called when the lifecycle node
        * enters the "shutting down" state.
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State& previous_state);

        /// Transition callback for state configuring
        /**
        * on_configure callback is being called when the lifecycle node
        * enters the "configuring" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "unconfigured".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State&);

        /// Transition callback for state activating
        /**
        * on_activate callback is being called when the lifecycle node
        * enters the "activating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "active" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "active"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        * 
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State&);

        /// Transition callback for state deactivating
        /**
        * on_deactivate callback is being called when the lifecycle node
        * enters the "deactivating" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "inactive" state or stays
        * in "active".
        * TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
        * TRANSITION_CALLBACK_FAILURE transitions to "active"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State&);

        /// Transition callback for state cleaningup
        /**
        * on_cleanup callback is being called when the lifecycle node
        * enters the "cleaningup" state.
        * Depending on the return value of this function, the state machine
        * either invokes a transition to the "unconfigured" state or stays
        * in "inactive".
        * TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
        * TRANSITION_CALLBACK_FAILURE transitions to "inactive"
        * TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
        */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State&);








private:


//##############Task2################   
//create CAN channel object
ros2socketcan canChannel0;
//create Publisher
    rclcpp::QoS qos{10};
    std::string pub_marker_array_topic_name = "/ars408/marker_array";
    std::string pub_object_list_topic_name = "/ars408/objectlist";
    std::string pub_tf_topic_name = "/tf";
    std::string frame_id_ = "/radar_link";

    rclcpp_lifecycle::LifecyclePublisher<radar_conti_ars408_msgs::msg::ObjectList>::SharedPtr object_list_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
//create can_receive_callback
    void can_receive_callback(const can_msgs::msg::Frame);
//create handle_object_list
    void handle_object_list(const can_msgs::msg::Frame);
//create publish_object_map
    void publish_object_map();
//create map container for object list
    std::map<int,radar_conti_ars408_msgs::msg::Object> object_map_;

//create data structures for radar object list
    radar_conti_ars408_msgs::msg::ObjectList object_list_;

//additional variables
    int operation_mode_;

// ##################################







};

}  // namespace radar_conti_ars408

#endif  // COMPOSITION__RADAR_CONTI_ARS408_COMPONENT_HPP_
