#include <iostream>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "auto_aim_interfaces/msg/serial.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

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

    void get_all_parameter();

    void parameters_init();

    int no_resistance_model();

    int single_resistance_model();

    int single_resistance_model_two();

    bool is_solvable();

    int solve_trajectory();

    void test();

    void target_callback(const auto_aim_interfaces::msg::Target msg);

    void angle_callback(const auto_aim_interfaces::msg::Serial msg);

    // parameters
    //------------------
    float object_z; // m
    float object_y; // m
    float object_x; // m
    float v_hero; // m/s
    float v_bubin;
    float v0;
    float distance; // m
    float angle_pitch;
    float angle_yaw;
    float alpha;
    float fly_t; // m
    float bias_t;

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
    vector<result> results;
    //------------------
    //------------------
    float randa_small;
    float randa_big;
    float randa;
    bool is_hero;
    float z_bias;
    float y_bias;
    //------------------
    //------------------
    // Subsciption
    //------------------
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Serial>::SharedPtr angle_sub_;
    //------------------
    // Publisher
    //------------------
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr maker_pub_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Serial>::SharedPtr result_pub_;
    //------------------
    //timer
    //------------------
    rclcpp::TimerBase::SharedPtr timer_;
    //------------------
};