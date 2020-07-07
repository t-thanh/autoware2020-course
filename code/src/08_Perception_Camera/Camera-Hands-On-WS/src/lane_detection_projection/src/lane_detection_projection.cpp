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
#include <opencv2/opencv.hpp>
#include "lane_detection_projection/matrixDefines.h"
#include "lane_msgs/msg/lane_marking.hpp"
#include "lane_msgs/msg/lane_marking_array_both.hpp"
#include "lane_msgs/msg/lane_marking_projected.hpp"
#include "lane_msgs/msg/lane_marking_projected_array_both.hpp"
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include <vector>

using std::placeholders::_1;


class lane_detection_projection : public rclcpp::Node
{

public:
    lane_detection_projection()
        : Node("lane_detection_projection")
    {
        //TASK 1: create publisher
        //TASK 1: create subscriber
    }

    //TASK 1: create callback function
    {
        //TASK 2: define message for the resulting projected lanes
        //TASK 2: check if left lane empty
        {
            //TASK 2: create container cv::Mat to be passed to undistortPoints
            //TASK 2: crete destination cv::Mat in which results will be stored
            //TASK 2: fill the container with points from left lane cv::Mat
            //TASK 2: call the undistortPoints function
            //TASK 2: iterate through undistorted points
            {
                //TASK 2: create matrix cv::Mat from undistorted point, make it homogenous
                //TASK 2: multiply with inverse of rotation matrix
                //TASK 2: normalize the homogenous resulting vector
                //TASK 2: scale ray to the propotion of z-transform-coordinate to put it in the x-y-plane(i.e. the ground)and apply transformation of transform vector
                //TASK 2: store projected point in appropriate message type
                //TASK 2: add point to vector of projected points for the left lane in the result message
            }
        }

        //TASK 2: check if right lane empty
        {
            //TASK 2: create container cv::Mat to be passed to undistortPoints
            //TASK 2: crete destination cv::Mat in which results will be stored
            //TASK 2: fill the container with points from right lane cv::Mat
            //TASK 2: call the undistortPoints function
            //TASK 2: iterate through undistorted points
            {
                //TASK 2: create matrix cv::Mat from undistorted point, make it homogenous
                //TASK 2: multiply with inverse of rotation matrix
                //TASK 2: normalize the homogenous resulting vector
                //TASK 2: scale ray to the propotion of z-transform-coordinate to put it in the x-y-plane(i.e. the ground) and apply transformation of transform vector
                //TASK 2: store projected point in appropriate message type
                //TASK 2: add point to vector of projected points for the left lane in the result message
            }
        }
        //Task 2: publish the resulting combined vectors
    }

private:
    //TASK 1: define publisher
    //TASK 1: define subscribtion
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lane_detection_projection>();
    rclcpp::Rate loop_rate(1/100.0);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    return 0;
}
