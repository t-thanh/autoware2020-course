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
        //TASK 1: create publisher for visualization messages
        //create subscriber for projected points    
    }

    void laneMarkingCallback(lane_msgs::msg::LaneMarkingProjectedArrayBoth::SharedPtr msg)
    {
        //TASK 3: create marker of type LINE_STRIP for left lane
        //TASK 3: fill the "points" field in the marker with the coordinates of recieved projected left lanemarkings
        //TASK 3: create marker of type LINE_STRIP for right lane
        //TASK 3: fill the "points" field in the marker with the coordinates of recieved projected right lanemarkings
        //TASK 3: publish the markers
        //Bonus 
    }

private:
    //TASK 1: define publisher
    //TASK 1: define subscription
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
