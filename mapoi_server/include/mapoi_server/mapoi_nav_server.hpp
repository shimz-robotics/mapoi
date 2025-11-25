#ifndef MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
#define MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "mapoi_interfaces/srv/get_tagged_pois.hpp"
#include "mapoi_interfaces/srv/get_route_pois.hpp"

#include <vector>
#include <memory>

class MapoiNavServer : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  explicit MapoiNavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // サービスコールバック
  void on_route_received(rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedFuture future);
  
  void send_waypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints);
  
  // Action Callbacks
  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleFollowWaypoints::SharedPtr, const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
  void result_callback(const GoalHandleFollowWaypoints::WrappedResult & result);

  // Clients
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetRoutePois>::SharedPtr route_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

#endif  // MAPOI_SERVER__MAPOI_NAV_SERVER_HPP_
