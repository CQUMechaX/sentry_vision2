#include <iostream>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "auto_aim_interfaces/msg/receive_serial.hpp"
#include "auto_aim_interfaces/msg/send_serial.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std;

#define g 9.8
#define ARMOR_NUM_BALANCE 2;
#define ARMOR_NUM_OUTPOST 3;

struct result
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
};

class Trajectoryer : public::rclcpp::Node
{
public:
    // hanshu
    Trajectoryer();

    void parameters_init();

    int no_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0);

    int single_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    int single_resistance_model_two(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    int two_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa);

    bool is_solvable(const float &object_x,const float &object_y,const float &object_z,const float &v0,float &alpha);

    int solve_trajectory();

    void test();

    void righttarget_callback(const auto_aim_interfaces::msg::Target msg);

    void rightangle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg);

    void get_need_pose(const float &object_x,const float &object_y,const float &object_z);

    // parameters
    //------------------
    float v0; // m/s
    float angle_pitch;
    float angle_yaw;
    float fly_t; // m
    float bias_t;
    float y_bias;
    float z_bias;
    //------------------
    float now_pitch;
    float now_yaw;
    bool is_shoot;
    //------------------
    //------------------
    int armor_num;
    // 敌方云台中心在我方云台中心坐标系下的坐标
    float yaw;
    float v_yaw;
    float vx;
    float ros_x;
    float vy;
    float ros_y;
    float vz;
    float ros_z;
    float r_1;
    float r_2;
    float dz;
    bool is_tracking;
    string id;
    //------------------
    //------------------
    float randa;
    bool is_hero;
    //------------------
    //------------------
    // Subsciption
    //------------------
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr righttarget_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::ReceiveSerial>::SharedPtr rightangle_sub_;
    //------------------
    // Publisher
    //------------------
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rightmaker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::SendSerial>::SharedPtr result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rightneedpose_pub_;
    //------------------
    //timer
    //------------------
    rclcpp::TimerBase::SharedPtr timer_;
    //------------------
};
