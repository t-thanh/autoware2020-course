/*  
Copyright 2020 Gjorgji Nikolovski

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/
#define ubyte unsigned char
#define uword unsigned short int
#define BOOST_BIND_NO_PLACEHOLDERS
#include <chrono>
#include <memory>
#include "lane_msgs/msg/lane_marking_projected_array_both.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include <vector>

using std::placeholders::_1;

class lane_detection_visualization : public rclcpp::Node
{

public:
    lane_detection_visualization()
        : Node("lane_detection_visualization")
    {
        pub_marker = create_publisher<visualization_msgs::msg::Marker>("/visual_feedback", 10);
        sub_markings = create_subscription<lane_msgs::msg::LaneMarkingProjectedArrayBoth>("/lane_markings_left_right_projected", 10, std::bind(&lane_detection_visualization::laneMarkingCallback, this, _1));
        
    }

    void laneMarkingCallback(lane_msgs::msg::LaneMarkingProjectedArrayBoth::SharedPtr msg)
    {
        visualization_msgs::msg::Marker ma_left_lane;
        visualization_msgs::msg::Marker ma_right_lane;
        

        ma_left_lane.header.stamp = this->now();
        ma_left_lane.header.frame_id = "map";
        ma_left_lane.id = 0;
        ma_left_lane.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ma_left_lane.action = visualization_msgs::msg::Marker::ADD;
        ma_left_lane.pose.position.x = 0;
        ma_left_lane.pose.position.y = 0;
        ma_left_lane.pose.position.z = 0;
        ma_left_lane.pose.orientation.x = 0;
        ma_left_lane.pose.orientation.y = 0;
        ma_left_lane.pose.orientation.z = 0;
        ma_left_lane.pose.orientation.w = 1;
        ma_left_lane.color.a=1.0;
        ma_left_lane.color.r=1.0;
        ma_left_lane.color.g=0;
        ma_left_lane.color.b=0;
        ma_left_lane.scale.x=0.1;
        ma_left_lane.scale.y=0.1;
        ma_left_lane.scale.z=0.1;

        for (int i = 0; i < msg->markings_left.size(); i++)
        {
            geometry_msgs::msg::Point point;
            point.x = msg->markings_left[i].x;
            point.y = msg->markings_left[i].y;
            point.z = msg->markings_left[i].z;
            ma_left_lane.points.push_back(point);
        }

        ma_right_lane.header.stamp = this->now();
        ma_right_lane.header.frame_id = "map";
        ma_right_lane.id = 1;
        ma_right_lane.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ma_right_lane.action = visualization_msgs::msg::Marker::ADD;
        ma_right_lane.pose.position.x = 0;
        ma_right_lane.pose.position.y = 0;
        ma_right_lane.pose.position.z = 0;
        ma_right_lane.pose.orientation.x = 0;
        ma_right_lane.pose.orientation.y = 0;
        ma_right_lane.pose.orientation.z = 0;
        ma_right_lane.pose.orientation.w = 1;
        ma_right_lane.color.a=1.0;
        ma_right_lane.color.r=1.0;
        ma_right_lane.color.g=0;
        ma_right_lane.color.b=0;
        ma_right_lane.scale.x=0.1;
        ma_right_lane.scale.y=0.1;
        ma_right_lane.scale.z=0.1;

        for (int i = 0; i < msg->markings_right.size(); i++)
        {
            geometry_msgs::msg::Point point;
            point.x = msg->markings_right[i].x;
            point.y = msg->markings_right[i].y;
            point.z = msg->markings_right[i].z;
            ma_right_lane.points.push_back(point);
        }

        pub_marker->publish(ma_left_lane);
        pub_marker->publish(ma_right_lane);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.id = 911;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_resource = "file://"+ament_index_cpp::get_package_share_directory("lane_detection_visualization")+"/resources/low_poly_911.dae";
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 1.0;
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.b = 1.0;
        marker.color.g = 1.0;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        pub_marker->publish(marker);
    }

private:
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_marker;
    std::shared_ptr<rclcpp::Subscription<lane_msgs::msg::LaneMarkingProjectedArrayBoth>> sub_markings;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lane_detection_visualization>();
    rclcpp::Rate loop_rate(1/100.0);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
