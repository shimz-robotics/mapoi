#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

#include "mapoi_interfaces/srv/get_map_info.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // if (argc != 2) {
  //   RCLCPP_INFO(rclcpp::get_logger("get_map_info_client"), "usage: get_map_info_client building_name");
  //   return 1;
  // }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_map_info_client");
  rclcpp::Client<mapoi_interfaces::srv::GetMapInfo>::SharedPtr client =
    node->create_client<mapoi_interfaces::srv::GetMapInfo>("get_map_info");

  auto request = std::make_shared<mapoi_interfaces::srv::GetMapInfo::Request>();
  // request->list_name = argv[1];

  while(!client->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("get_map_info_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("get_map_info_client"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto map_info = result.get();
    RCLCPP_INFO(rclcpp::get_logger("get_map_info_client"), "Current List: %s", map_info->maps_path.c_str());
    RCLCPP_INFO(rclcpp::get_logger("get_map_info_client"), "Current Map: %s", map_info->map_name.c_str());
    for(auto map : map_info->maps_list){
      RCLCPP_INFO(rclcpp::get_logger("get_map_info_client"), "Map: %s", map.c_str());
    }
  }else{
    RCLCPP_ERROR(rclcpp::get_logger("get_map_info_client"), "Failed to call service get_poi");
  }

  rclcpp::shutdown();
  return 0;
}