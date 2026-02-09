#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

#include "mapoi_interfaces/srv/get_tagged_pois.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    RCLCPP_INFO(rclcpp::get_logger("get_tagged_pois_client"), "usage: get_tagged_pois_client destination");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("get_tagged_pois_client");
  rclcpp::Client<mapoi_interfaces::srv::GetTaggedPois>::SharedPtr client =
    node->create_client<mapoi_interfaces::srv::GetTaggedPois>("get_tagged_pois");

  auto request = std::make_shared<mapoi_interfaces::srv::GetTaggedPois::Request>();
  request->tag = argv[1];

  while(!client->wait_for_service(1s)){
    if(!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("get_tagged_pois_client"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("get_tagged_pois_client"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto tag_poi = result.get()->pois_list;
    RCLCPP_INFO(rclcpp::get_logger("get_tagged_pois_client"), "Size of result: %ld", tag_poi.size());
    for(auto poi : tag_poi){
      RCLCPP_INFO(rclcpp::get_logger("get_tagged_pois_client"), "POI Name: %s", poi.name.c_str());
    }
  }else{
    RCLCPP_ERROR(rclcpp::get_logger("get_tagged_pois_client"), "Failed to call service get_tagged_pois");
  }

  rclcpp::shutdown();
  return 0;
}