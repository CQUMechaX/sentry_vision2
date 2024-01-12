#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <memory>
#include <iostream>
#include <thread>
#include <string>
#include <unistd.h>
#include <vector>
#include <fstream>
#include "rm_decision/commander.hpp"

using namespace std::chrono_literals;

namespace rm_decision
{

   Commander::Commander(const rclcpp::NodeOptions & options) : Node("commander",options)
   {
      RCLCPP_INFO(this->get_logger(), "Commander node has been started.");
      
      //创建客户端
      nav_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"navigate_to_pose");
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
      send_goal_options.goal_response_callback = std::bind(&Commander::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&Commander::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&Commander::result_callback, this, std::placeholders::_1);
      //初始化状态
      loadNavPoints();
      RCLCPP_INFO(this->get_logger(), "导航点个数: %ld",nav_points_.size());
      random = nav_points_.begin();
      goal = nav_points_[0];
      sleep(10);
      currentState = std::make_shared<PatrolState>(this);
      // 创建订阅(订阅裁判系统的信息)
      hp_sub_ = this->create_subscription<rm_decision_interfaces::msg::AllRobotHP>(
         "all_robot_hp", 10, std::bind(&Commander::hp_callback, this, std::placeholders::_1));
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
         "odom", 10, std::bind(&Commander::pose_callback, this, std::placeholders::_1));
      // 创建线程（处理信息和发布命令）
      
      commander_thread_ = std::thread(&Commander::decision, this);
      executor_thread_ = std::thread(&Commander::executor, this);

   }

   Commander::~Commander(){
      if(commander_thread_.joinable()){
         commander_thread_.join();
      }
         
      if(executor_thread_.joinable()){
         executor_thread_.join();
      }

   }
   // 处理信息（还未写）
   void Commander::decision(){
      while (rclcpp::ok())
      {
         // 读取信息
         // 判断信息
         // 改变状态
         setState(std::make_shared<PatrolState>(this));
         // 发布命令
         // 休眠
      }

   }

   // 执行器线程
   void Commander::executor(){
      while (rclcpp::ok())
      {
         currentState->handle();
         // RCLCPP_INFO(this->get_logger(), "1");
         // if(auto nextState = currentState->check()){
         //    setState(nextState.value());
         // }
      }
   }

   // 改变状态
   void Commander::setState(std::shared_ptr<State> state) {
      currentState = state;
   }

   
   
   // 导航到点
   void Commander::nav_to_pose(geometry_msgs::msg::Pose goal_pose){
      nav_to_pose_client->wait_for_action_server();
      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose.pose = goal_pose;
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = this->now();
      goal_msg.behavior_tree = "";

      send_goal_future = nav_to_pose_client->async_send_goal(goal_msg,send_goal_options);
   

   }

   // 请求反馈
   void Commander::goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future){
      auto goal_handle = future.get();
      if (!goal_handle) {
         RCLCPP_INFO(this->get_logger(),"Goal was rejected by server");
         checkgoal = true;
         return;
      }
      else{
         RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
      }
   }
   
   // 过程反馈
   void Commander::feedback_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
   ){
      // RCLCPP_INFO(this->get_logger(),"Received feedback: 去往目标点的距离: %.2f m",feedback->distance_remaining);
   }
   
   // 结果反馈
   void Commander::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result){
      switch (result.code) {
         case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),"Goal was reached!");
            checkgoal = true;
            break;
         case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(),"Goal was aborted");
            checkgoal = true;
            break;
         case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(),"Goal was canceled");
            checkgoal = true;
            break;
         default:
            RCLCPP_INFO(this->get_logger(),"Unknown result code");
            checkgoal = true;
            break;
      }
   }
   
   // 读取导航点
   void Commander::loadNavPoints() {
         geometry_msgs::msg::Pose pose;
         std::vector<double> pose_list;
         this->declare_parameter("poses_list", pose_list);
         auto pose_param = this->get_parameter("poses_list").as_double_array();
         RCLCPP_INFO(this->get_logger(), "开始传入导航点");
         RCLCPP_INFO(this->get_logger(), "导航点个数: %ld",pose_param.size()/3);
         for(uint i = 0; i < pose_param.size(); i=i+3){
         pose.position.x = pose_param[i];
         pose.position.y = pose_param[i+1];
         pose.position.z = pose_param[i+2];
         pose.orientation.x = 0.0;
         pose.orientation.y = 0.0; 
         pose.orientation.z = 0.0;
         pose.orientation.w = 1.0;
         nav_points_.push_back(pose);
         RCLCPP_INFO(this->get_logger(), "传入第%d个导航点: %.2f, %.2f, %.2f",(i+3)/3,pose.position.x,pose.position.y,pose.position.z);
         }
   }
   
   // 订阅回调
   void Commander::hp_callback(const rm_decision_interfaces::msg::AllRobotHP::SharedPtr msg) {
      red_1_robot_hp = msg->red_1_robot_hp;
      red_2_robot_hp = msg->red_2_robot_hp;
      red_3_robot_hp = msg->red_3_robot_hp;
      red_4_robot_hp = msg->red_4_robot_hp;
      red_5_robot_hp = msg->red_5_robot_hp;
      red_7_robot_hp = msg->red_7_robot_hp;
      red_outpost_hp = msg->red_outpost_hp;
      red_base_hp = msg->red_base_hp;
      blue_1_robot_hp = msg->blue_1_robot_hp;
      blue_2_robot_hp = msg->blue_2_robot_hp;
      blue_3_robot_hp = msg->blue_3_robot_hp;
      blue_4_robot_hp = msg->blue_4_robot_hp;
      blue_5_robot_hp = msg->blue_5_robot_hp;
      blue_7_robot_hp = msg->blue_7_robot_hp;
      blue_outpost_hp = msg->blue_outpost_hp;
      blue_base_hp = msg->blue_base_hp;
      }
   
   void Commander::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      currentpose = msg;
   }

   // 巡逻模式
   void PatrolState::handle() {
      if(commander->checkgoal){
         commander->nav_to_pose(*commander->random);
         commander->random++;
         if(commander->random == commander->nav_points_.end()){
            commander->random = commander->nav_points_.begin();
         }
         commander->checkgoal = false;
      }
   }

   // goandstay模式(可用于守卫某点或逃跑等)
   void GoAndStayState::handle() {
      if(commander->checkgoal){
         commander->nav_to_pose(commander->goal);
         commander->checkgoal = false;
      }
   }
   


} // namespace rm_decision
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_decision::Commander)