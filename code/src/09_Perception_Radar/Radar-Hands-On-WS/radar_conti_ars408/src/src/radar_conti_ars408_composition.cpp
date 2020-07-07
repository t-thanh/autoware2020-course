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

#include <memory>

#include "../include/radar_conti_ars408_component.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // This will also parse command line arguments one day.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;


  // Add some nodes to the executor which provide work for the executor during its "spin" function.
  // An example of available work is executing a subscription callback, or a timer callback.
  auto radar_conti_ars408_node = std::make_shared<FHAC::radar_conti_ars408>(options);
  exec.add_node(radar_conti_ars408_node->get_node_base_interface());


  // spin will block (https://de.wikipedia.org/wiki/Spinlock) until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
