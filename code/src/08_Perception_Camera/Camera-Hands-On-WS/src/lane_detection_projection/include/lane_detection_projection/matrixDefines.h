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
#include <opencv2/opencv.hpp>
float rotation_values[] = {-0.9984952,  0.0481957,  0.0261626,
                             0.0226059, -0.0729187,  0.9970816,
                             0.0499628,  0.9961727,  0.0717195};
cv::Mat_<float> rotation_matrix(3, 3, rotation_values);
float transform_values[] = {-0.30499256,
                            -1.7974359,
                            1.4319205};
cv::Mat_<float> transform_matrix(1, 3, transform_values);
float intrinsic_values[] = {1662, 0, 960.00000,
                            0, 1632, 604.00000, 
                            0,    0,         1};
cv::Mat_<float> intrinsic_calibration_matrix(3, 3, intrinsic_values);