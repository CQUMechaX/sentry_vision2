//作出决策的节点
#ifndef RM_DECISION__COMMANDER_HPP_
#define RM_DECISION__COMMANDER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <iostream>
#include <vector>
#include <thread>
#include <optional>
#include <fstream>
#include <string>

#include "rm_decision_interfaces/msg/all_robot_hp.hpp"

namespace rm_decision{

class Commander;

class State {
private:
public:
  State(Commander* commander) : commander(commander) {}
  virtual ~State() = default;
  virtual void handle() {};
  
  Commander* commander;
  // virtual std::optional<std::shared_ptr<State>> check() = 0;
};




class Commander : public rclcpp::Node
{
public:
  explicit Commander(const rclcpp::NodeOptions & options); // 重载构造函数

  ~Commander() override; // 析构函数
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;

  void nav_to_pose(geometry_msgs::msg::Pose goal_pose);

  void goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

  void feedback_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
  );

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

  double distence(geometry_msgs::msg::PoseStamped::SharedPtr a, geometry_msgs::msg::PoseStamped::SharedPtr b );
  
  geometry_msgs::msg::PoseStamped::SharedPtr currentpose;
  
  std::vector<geometry_msgs::msg::Pose> nav_points_;
  
  std::vector<geometry_msgs::msg::Pose>::iterator random;
  geometry_msgs::msg::Pose goal;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> send_goal_future;

  bool checkgoal = true;
  
  // 机器人血量
  uint red_1_robot_hp;
  uint red_2_robot_hp;
  uint red_3_robot_hp;
  uint red_4_robot_hp;
  uint red_5_robot_hp;
  uint red_7_robot_hp;
  uint red_outpost_hp;
  uint red_base_hp;

  uint blue_1_robot_hp;
  uint blue_2_robot_hp;
  uint blue_3_robot_hp;
  uint blue_4_robot_hp;
  uint blue_5_robot_hp;
  uint blue_7_robot_hp;
  uint blue_outpost_hp;
  uint blue_base_hp;

  // 敌方机器人坐标
  geometry_msgs::msg::PoseStamped::SharedPtr enemypose;

  private:
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;
  std::thread commander_thread_;
  std::thread executor_thread_;
  void decision();
  void executor();
  void setState(std::shared_ptr<State> state);
  void loadNavPoints();
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void hp_callback(const rm_decision_interfaces::msg::AllRobotHP::SharedPtr msg);

  std::shared_ptr<State> currentState;

  rclcpp::Subscription<rm_decision_interfaces::msg::AllRobotHP>::SharedPtr hp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

class PatrolState : public State {
  public:
    explicit PatrolState(Commander* commander) : State(commander) {}
    void handle() override;
};

class GoAndStayState : public State {
  public:
    explicit GoAndStayState(Commander* commander) : State(commander) {}
    void handle() override;
};

class AttackState : public State {
  public:
    explicit AttackState(Commander* commander) : State(commander) {}
    virtual void handle() override;
};

}

#endif // RM_DECISION__COMMANDER_HPP_