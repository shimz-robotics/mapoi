#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

#include "mapoi_interfaces/srv/switch_map.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2){
    RCLCPP_INFO(rclcpp::get_logger("switch_map_client"), "usage: switch_map_client giken3F_sim");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("switch_map_client");
  rclcpp::Client<mapoi_interfaces::srv::SwitchMap>::SharedPtr client =
    node->create_client<mapoi_interfaces::srv::SwitchMap>("switch_map");

  auto request = std::make_shared<mapoi_interfaces::srv::SwitchMap::Request>();
  // request->map_name = "giken3F_sim";
  request->map_name = argv[1];

  while(!client->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("switch_map"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("switch_map"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if(rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("switch_map"), "Success: %d", result.get()->success);
  }else{
    RCLCPP_ERROR(rclcpp::get_logger("switch_map"), "Failed to call service switch_map");
  }

  rclcpp::shutdown();
  return 0;
}