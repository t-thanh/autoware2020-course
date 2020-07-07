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
    object_list_publisher_ = this->create_publisher<radar_conti_ars408_msgs::msg::ObjectList>(pub_object_list_topic_name, qos);
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(pub_tf_topic_name, qos);
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_marker_array_topic_name, qos);

// Initialise CAN subscriber
    
    canChannel0.Init("can0", std::bind(&radar_conti_ars408::can_receive_callback, this, _1));
 
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
    
    object_list_publisher_->on_activate();
    tf_publisher_->on_activate();
    marker_array_publisher_->on_activate();

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


            if (msg.id == ID_RadarState) {
                operation_mode_ =CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(msg.data),1.0);

            }

            //no output
            if (operation_mode_ == 0x00) {

                return;
            }

            //object list
            if (operation_mode_ == 0x01) {
                handle_object_list(msg);
               
            }

// ##################################

}


void radar_conti_ars408::handle_object_list(const can_msgs::msg::Frame msg) {

//##############Task6################   
// create Object Handler
// 1. wait for CAN Status Message
// 2. publish old object data and fill object_list_header, object_count and clear object_map
// 3. wait for CAN Message General, Quality and Extended and copy values to the new object_map 
// ##################################

            //Object List Status
            //if there is an new update for the objectlist, publish the recent data and update time stamp for
            //the new incoming data
            if (msg.id == ID_Obj_0_Status) {
  

                publish_object_map();

                object_list_.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
                object_list_.object_count.data = GET_Obj_0_Status_Obj_NofObjects(msg.data);

                object_map_.clear();


            }


            //Object General Information
            //for each Obj_1_General message a new object has to be created in the map
            if (msg.id == ID_Obj_1_General) {

                radar_conti_ars408_msgs::msg::Object o;

                //object ID
                int id = GET_Obj_1_General_Obj_ID(msg.data);
                o.obj_id.data = GET_Obj_1_General_Obj_ID(msg.data);

               // //RCLCPP_INFO(this->get_logger(), "Object_ID: 0x%04x", o.obj_id.data);

                //longitudinal distance
                o.object_general.obj_distlong.data =
                        CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(msg.data), 1.0);

                //lateral distance
                o.object_general.obj_distlat.data =
                        CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(msg.data), 1.0);

                //relative longitudinal velocity
                o.object_general.obj_vrellong.data =
                        CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(msg.data), 1.0);

                //relative lateral velocity
                o.object_general.obj_vrellat.data =
                        CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(msg.data), 1.0);

                o.object_general.obj_dynprop.data =
                        CALC_Obj_1_General_Obj_DynProp(GET_Obj_1_General_Obj_DynProp(msg.data), 1.0);

                o.object_general.obj_rcs.data = 
                        CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(msg.data), 1.0);

                //insert object into map
                object_map_.insert(std::pair<int, radar_conti_ars408_msgs::msg::Object>(id, o));
            }

            //Object Quality Information
            //for each Obj_2_Quality message the existing object in the map has to be updated
            if (msg.id == ID_Obj_2_Quality) {

               // //RCLCPP_INFO(this->get_logger(), "Received Object_2_Quality msg (0x60c)");

                int id = GET_Obj_2_Quality_Obj_ID(msg.data);


                object_map_[id].object_quality.obj_distlong_rms.data =
                        CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(msg.data), 1.0);

                object_map_[id].object_quality.obj_distlat_rms.data =
                        CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(msg.data), 1.0);

                object_map_[id].object_quality.obj_vrellong_rms.data =
                        CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(msg.data), 1.0);
 
                object_map_[id].object_quality.obj_vrellat_rms.data =
                        CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(msg.data), 1.0);

            }

            //Object Extended Information
            //for each Obj_3_ExtInfo message the existing object in the map has to be updated
            if (msg.id == ID_Obj_3_Extended) {


                int id = GET_Obj_3_Extended_Obj_ID(msg.data);


                object_map_[id].object_extended.obj_arellong.data =
                        CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(msg.data), 1.0);

                object_map_[id].object_extended.obj_arellat.data =
                        CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(msg.data), 1.0);

                object_map_[id].object_extended.obj_class.data =
                        CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

                object_map_[id].object_extended.obj_orientationangle.data =
                        CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(msg.data),
                                                                 1.0);

                object_map_[id].object_extended.obj_length.data =
                        CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(msg.data), 1.0);

                object_map_[id].object_extended.obj_width.data =
                        CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(msg.data), 1.0);
            }

// ##################################

}

void radar_conti_ars408::publish_object_map() {

//##############Task6################   
// create function to publish marker
// 1. create transforms
// 2. publish transforms & Objects


                tf2_msgs::msg::TFMessage transforms;

                std::map<int, radar_conti_ars408_msgs::msg::Object>::iterator itr;

                for (itr = object_map_.begin(); itr != object_map_.end(); ++itr) {

                    std::string tf_name = "object_" + std::to_string(itr->first);

                    geometry_msgs::msg::TransformStamped tf;

                    tf.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
                    tf.header.frame_id = frame_id_;

                    tf.child_frame_id = tf_name;

                    tf.transform.translation.x = itr->second.object_general.obj_distlong.data;
                    tf.transform.translation.y = itr->second.object_general.obj_distlat.data;
                    tf.transform.translation.z = 1.0;

                    tf.transform.rotation.w = 1.0;
                    tf.transform.rotation.x = 0.0;
                    tf.transform.rotation.y = 0.0;
                    tf.transform.rotation.y = 0.0;

                    transforms.transforms.push_back(tf);


                    object_list_.objects.push_back(itr->second); 

                }


                object_list_publisher_->publish(object_list_);
                tf_publisher_->publish(transforms);


// 3. create marker array
// 4. create marker
// 5. delete old marker (marker.action=3 and publish)
// 6. create marker for each object



                visualization_msgs::msg::MarkerArray marker_array;
                marker_array.markers.clear();

                //delete old marker
                visualization_msgs::msg::Marker ma;
                ma.action=3;
                marker_array.markers.push_back(ma);
                marker_array_publisher_->publish(marker_array);
                marker_array.markers.clear();

                //marker for ego car
                visualization_msgs::msg::Marker mEgoCar;

                mEgoCar.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
                mEgoCar.header.frame_id = frame_id_;
                mEgoCar.ns = "";
                mEgoCar.id = 999;

                //if you want to use a cube comment out the next 2 lines
                mEgoCar.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                mEgoCar.mesh_resource = "file://"+ament_index_cpp::get_package_share_directory("radar_conti_ars408")+"/resources/low_poly_911.dae";
                // mEgoCar.type = 1; // cube
                mEgoCar.action = 0; // add/modify
                mEgoCar.pose.position.x = -2.0;
                mEgoCar.pose.position.y = 0.0;
                mEgoCar.pose.position.z = 1.0;

                tf2::Quaternion myQuaternion;
                myQuaternion.setRPY(0, 0, M_PI/2);

                mEgoCar.pose.orientation.w = myQuaternion.getW();
                mEgoCar.pose.orientation.x = myQuaternion.getX();
                mEgoCar.pose.orientation.y = myQuaternion.getY();
                mEgoCar.pose.orientation.z = myQuaternion.getZ();
                mEgoCar.scale.x = 1.0;
                mEgoCar.scale.y = 1.0;
                mEgoCar.scale.z = 1.0;
                mEgoCar.color.r = 0.0;
                mEgoCar.color.g = 0.0;
                mEgoCar.color.b = 1.0;
                mEgoCar.color.a = 1.0;
                mEgoCar.lifetime = rclcpp::Duration(0.2);
                mEgoCar.frame_locked = false;

                marker_array.markers.push_back(mEgoCar);

                for (itr = object_map_.begin(); itr != object_map_.end(); ++itr) {
                    visualization_msgs::msg::Marker mobject;
                    visualization_msgs::msg::Marker mtext;

                    mtext.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
                    mtext.header.frame_id = frame_id_;
                    mtext.ns = "";
                    mtext.id = (itr->first+100);
                    mtext.type = 1; //Cube
                    mtext.action = 0; // add/modify
                    mtext.pose.position.x = itr->second.object_general.obj_distlong.data;
                    mtext.pose.position.y = itr->second.object_general.obj_distlat.data;
                    mtext.pose.position.z = 4.0;

                
                    //myQuaternion.setRPY(M_PI / 2, 0, 0);
                    myQuaternion.setRPY(0, 0, 0);

                    mtext.pose.orientation.w = myQuaternion.getW();
                    mtext.pose.orientation.x = myQuaternion.getX();
                    mtext.pose.orientation.y = myQuaternion.getY();
                    mtext.pose.orientation.z = myQuaternion.getZ();
                    mtext.scale.x = 1.0;
                    mtext.scale.y = 1.0;
                    mtext.scale.z = 2.0;
                    mtext.color.r = 1.0;
                    mtext.color.g = 1.0;
                    mtext.color.b = 1.0;
                    mtext.color.a = 1.0;
                    mtext.lifetime = rclcpp::Duration(0.2);
                    mtext.frame_locked = false;
                    mtext.type=9;
                    mtext.text= "object_" + std::to_string(itr->first) + ": \n" 
                    + " RCS: " + std::to_string(itr->second.object_general.obj_rcs.data) + "dBm^2" + " \n" 
                    + " V_long: " +   std::to_string(itr->second.object_general.obj_vrellong.data) + "m/s" + " \n" 
                    + " V_lat: " + std::to_string(itr->second.object_general.obj_vrellat.data) + "m/s" + " \n" 
                    + " Orientation: " + std::to_string(itr->second.object_extended.obj_orientationangle.data) + "degree";

                    marker_array.markers.push_back(mtext);



                    mobject.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
                    mobject.header.frame_id = frame_id_;
                    mobject.ns = "";
                    mobject.id = itr->first;
                    mobject.type = 1; //Cube
                    mobject.action = 0; // add/modify
                    mobject.pose.position.x = itr->second.object_general.obj_distlong.data;
                    mobject.pose.position.y = itr->second.object_general.obj_distlat.data;
                    mobject.pose.position.z = 1.0;

                    myQuaternion.setRPY(0, 0, 0);

                    mobject.pose.orientation.w = myQuaternion.getW();
                    mobject.pose.orientation.x = myQuaternion.getX();
                    mobject.pose.orientation.y = myQuaternion.getY();
                    mobject.pose.orientation.z = myQuaternion.getZ();
                    mobject.scale.x = itr->second.object_extended.obj_length.data;
                    mobject.scale.y = itr->second.object_extended.obj_width.data;
                    mobject.scale.z = 1.0;
                    mobject.color.r = 0.0;
                    mobject.color.g = 1.0;
                    mobject.color.b = 0.0;
                    mobject.color.a = 1.0;
                    mobject.lifetime = rclcpp::Duration(0.2);
                    mobject.frame_locked = false;

                    marker_array.markers.push_back(mobject);
                

                }
                marker_array_publisher_->publish(marker_array);
            
// ##################################

}


} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
//CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_ars408, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_ars408)
