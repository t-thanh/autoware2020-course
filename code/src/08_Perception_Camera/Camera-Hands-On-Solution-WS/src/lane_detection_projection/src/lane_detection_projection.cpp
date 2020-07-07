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

#include <stdio.h>
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
        pub_projected_markings = create_publisher<lane_msgs::msg::LaneMarkingProjectedArrayBoth>("/lane_markings_left_right_projected", 10);
        sub_markings = create_subscription<lane_msgs::msg::LaneMarkingArrayBoth>("/lane_markings_left_right", 10, std::bind(&lane_detection_projection::laneMarkingCallback, this, _1));
    }

    void laneMarkingCallback(lane_msgs::msg::LaneMarkingArrayBoth::SharedPtr msg)
    {
        
        lane_msgs::msg::LaneMarkingProjectedArrayBoth combined_lanes;
        if (msg->markings_left.size() > 0)
        {
            cv::Mat_<cv::Point2f> leftLanePointMatrix(1, msg->markings_left.size());
            cv::Mat undistortedLeftLanePointMatrixNormalised;
            for (int i = 0; i < msg->markings_left.size(); i++)
            {
                leftLanePointMatrix(i) = cv::Point2f(msg->markings_left[i].u, msg->markings_left[i].v);
            }
            undistortPoints(leftLanePointMatrix, undistortedLeftLanePointMatrixNormalised, intrinsic_calibration_matrix, cv::noArray());
            for (cv::MatIterator_<cv::Point2f> i = undistortedLeftLanePointMatrixNormalised.begin<cv::Point2f>(); i != undistortedLeftLanePointMatrixNormalised.end<cv::Point2f>(); ++i)
            {
                float norm[] = {(*i).x, (*i).y, 1};
                cv::Mat rotated_ray = cv::Mat_<float>(1, 3, norm) * rotation_matrix.inv();
                cv::Mat rotated_normalised_ray = rotated_ray / rotated_ray.at<float>(2);
                float xGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(0) + transform_matrix.at<float>(0);
                float yGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(1) + transform_matrix.at<float>(1);
                lane_msgs::msg::LaneMarkingProjected lmp;
                lmp.x = -xGround;
                lmp.y = yGround;
                lmp.z = 0;
                combined_lanes.markings_left.push_back(lmp);
            }
        }

        if (msg->markings_right.size() > 0)
        {
            cv::Mat_<cv::Point2f> rightLanePointMatrix(1, msg->markings_right.size());
            cv::Mat undistortedRightLanePointMatrixNormalised;
            for (int i = 0; i < msg->markings_right.size(); i++)
            {
                rightLanePointMatrix(i) = cv::Point2f(msg->markings_right[i].u, msg->markings_right[i].v);
            }

            undistortPoints(rightLanePointMatrix, undistortedRightLanePointMatrixNormalised, intrinsic_calibration_matrix, cv::noArray());

            for (cv::MatIterator_<cv::Point2f> i = undistortedRightLanePointMatrixNormalised.begin<cv::Point2f>(); i != undistortedRightLanePointMatrixNormalised.end<cv::Point2f>(); ++i)
            {
                float norm[] = {(*i).x, (*i).y, 1};
                cv::Mat rotated_ray = cv::Mat_<float>(1, 3, norm) * rotation_matrix.inv();
                cv::Mat rotated_normalised_ray = rotated_ray / rotated_ray.at<float>(2);
                float xGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(0) + transform_matrix.at<float>(0);
                float yGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(1) + transform_matrix.at<float>(1);
                lane_msgs::msg::LaneMarkingProjected lmp;
                lmp.x = -xGround;
                lmp.y = yGround;
                lmp.z = 0;
                combined_lanes.markings_right.push_back(lmp);
            }
        }
        pub_projected_markings->publish(combined_lanes);
    }

private:
    std::shared_ptr<rclcpp::Publisher<lane_msgs::msg::LaneMarkingProjectedArrayBoth>> pub_projected_markings;
    std::shared_ptr<rclcpp::Subscription<lane_msgs::msg::LaneMarkingArrayBoth>> sub_markings;
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
