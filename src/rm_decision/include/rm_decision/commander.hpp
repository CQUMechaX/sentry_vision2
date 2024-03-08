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
#include "nav_msgs/msg/odometry.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <iostream>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <thread>
#include <optional>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "rm_decision_interfaces/msg/all_robot_hp.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

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

class Robot {
public:
  Robot() = default;
  Robot(int id, geometry_msgs::msg::PoseStamped pose, uint hp) : id(id), pose(pose), hp(hp) {}
  int id;
  geometry_msgs::msg::PoseStamped pose;
  uint hp;
  bool attack = false;
  void check() {
    if (pose.pose.position.x >= 4.5 ) {
      attack = true;
    }
  }
};




class Commander : public rclcpp::Node
{
public:
  explicit Commander(const rclcpp::NodeOptions & options); // 重载构造函数

  ~Commander() override; // 析构函数
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;

  void nav_to_pose(geometry_msgs::msg::PoseStamped goal_pose);

  void goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

  void feedback_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
  );

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

  double distence(geometry_msgs::msg::PoseStamped a);

  void processPoses(std::vector<double>& pose_param,std::vector<geometry_msgs::msg::PoseStamped> nav_points_);

  void getcurrentpose();

  bool ifattack();
  
  geometry_msgs::msg::PoseStamped currentpose;
  
  std::vector<geometry_msgs::msg::PoseStamped> Patrol_points_;
  std::vector<geometry_msgs::msg::PoseStamped> Route1_points_;
  std::vector<geometry_msgs::msg::PoseStamped> Route2_points_;
  std::vector<geometry_msgs::msg::PoseStamped> Route3_points_;
  
  std::vector<geometry_msgs::msg::PoseStamped>::iterator random;
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped home;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> send_goal_future;

  bool checkgoal = true;
  bool start = false;
  
  // 机器人血量
  Robot self_1 = Robot(1, geometry_msgs::msg::PoseStamped(), 150);
  Robot self_2 = Robot(2, geometry_msgs::msg::PoseStamped(), 150);
  Robot self_3 = Robot(3, geometry_msgs::msg::PoseStamped(), 150);
  Robot self_4 = Robot(4, geometry_msgs::msg::PoseStamped(), 150);
  Robot self_5 = Robot(5, geometry_msgs::msg::PoseStamped(), 150);
  Robot self_7 = Robot(7, geometry_msgs::msg::PoseStamped(), 600);
  Robot enemy_1 = Robot(1, geometry_msgs::msg::PoseStamped(), 150);
  Robot enemy_2 = Robot(2, geometry_msgs::msg::PoseStamped(), 150);
  Robot enemy_3 = Robot(3, geometry_msgs::msg::PoseStamped(), 150);
  Robot enemy_4 = Robot(4, geometry_msgs::msg::PoseStamped(), 150);
  Robot enemy_5 = Robot(5, geometry_msgs::msg::PoseStamped(), 150);
  Robot enemy_7 = Robot(7, geometry_msgs::msg::PoseStamped(), 600);
  uint self_outpost_hp;
  uint self_base_hp;
  
  uint enemy_outpost_hp;
  uint enemy_base_hp;
  bool color; //true 为蓝
  int enemy_num = 0;
  uint enemyhp();

  // 敌方机器人坐标
  geometry_msgs::msg::PoseStamped enemypose;
  bool tracking = false;

  private:
  
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;
  std::thread commander_thread_;
  std::thread executor_thread_;
  void decision();
  void executor();
  void setState(std::shared_ptr<State> state);
  void loadNavPoints();
  void aim_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

  void hp_callback(const rm_decision_interfaces::msg::AllRobotHP::SharedPtr msg);
  

  std::shared_ptr<State> currentState;

  rclcpp::Subscription<rm_decision_interfaces::msg::AllRobotHP>::SharedPtr hp_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr aim_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  // tf2_ros::Buffer buffer{get_clock()};
  // tf2_ros::TransformListener tflistener{buffer};
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