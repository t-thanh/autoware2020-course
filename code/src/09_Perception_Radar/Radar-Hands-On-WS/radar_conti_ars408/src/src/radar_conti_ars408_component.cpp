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

#include "../include/radar_conti_ars408_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <math.h>

#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

namespace FHAC
{

// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
radar_conti_ars408::radar_conti_ars408(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("radar_conti_ars408", options)
{


}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_configure(
    const rclcpp_lifecycle::State&)
{

//##############Task3################   
// Initialise Publisher

// Initialise CAN subscriber
 
// ##################################


    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
}



rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_shutdown(
  const rclcpp_lifecycle::State& previous_state)
{

    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_error(
    const rclcpp_lifecycle::State& previous_state) 
{

    RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_activate(
    const rclcpp_lifecycle::State&) 
{

    //##############Task4################   
    // activate Publisher
    // 1. every publisher must be activated in the transistion state on_activate

    // ##################################

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_deactivate(
    const rclcpp_lifecycle::State&)
{

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_cleanup(
    const rclcpp_lifecycle::State&)
{
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}



void radar_conti_ars408::can_receive_callback(const can_msgs::msg::Frame msg)
{

//##############Task5################   
// create can receive callback
// 1. wait for Radar State Message
// 2. if Radar Output Mode ==1 in Radar State Message -> call handle_object_list function


// ##################################

}


void radar_conti_ars408::handle_object_list(const can_msgs::msg::Frame msg) {

//##############Task6################   
// create Object Handler
// 1. wait for CAN Status Message
// 2. publish old object data and fill object_list_header, object_count and clear object_map
// 3. wait for CAN Message General, Quality and Extended and copy values to the new object_map 
// ##################################


}

void radar_conti_ars408::publish_object_map() {

//##############Task6################   
// create function to publish marker
// 1. create transforms
// 2. publish transforms & Objects


// 3. create marker array
// 4. create marker
// 5. delete old marker (marker.action=3 and publish)
// 6. create marker for each object

            
// ##################################

}


} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
//CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_ars408, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_ars408)
