#include "trajectoryer.hpp"

inline float ft0(float &t0, float &randa, float &tan, float &z0, float &v0)
{
    return (g / randa) * ((1/randa) * log(1 - randa * t0) + t0) + tan - z0; 
}

inline float f_t0(float &t0, float &randa, float &tan, float &z0, float &v0)
{
    return (g / randa) * (-randa / randa / (1 - randa * t0) + 1) + (v0 * v0 * t0 / tan);
}

Trajectoryer::Trajectoryer() : 
    Node("trajectory")
{
    parameters_init();

    get_all_parameter();

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
                  "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Trajectoryer::target_callback, this, std::placeholders::_1));
    angle_sub_ = this->create_subscription<auto_aim_interfaces::msg::Serial>(
        "/angle/init", 10, std::bind(&Trajectoryer::angle_callback, this, std::placeholders::_1));
    
    maker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/aiming_point", 10);
    result_pub_ = this->create_publisher<auto_aim_interfaces::msg::Serial>(
        "/trajectory/result", 10);
  
   //timer_ = this->create_wall_timer(5s, std::bind(&Trajectoryer::test,this));
}

void  Trajectoryer::get_all_parameter()
{
    if(is_hero)
    {
        v0 = 15;
    }
    else 
    {
        v0 = 28;
    }
    bias_t = 0.1;
    z_bias = -0.042;
    y_bias = -0.060;
    fly_t = 0.5;
}

void  Trajectoryer::parameters_init()
{
    //----------------------------------------------------
    is_hero = false; // 根据情况自己修改，英雄大弹丸为1,步兵小弹丸为0
    //----------------------------------------------------
    //****************************************************
    float c = 0.5; 
    float p = 1.204; // kg/m3
    float d_small = 16.8 / 1000; // mm->m
    float d_big = 42.5 / 1000;  // mm->m   
    float m_small =  3.2 / 1000; // g->kg
    float m_big = 44.5 / 1000; // g->kg
    float s_small = M_PI * pow(d_small / 2, 2);
    float s_big = M_PI * pow(d_big / 2, 2);
    v0 = 16;
    randa_small =  (c * p * s_small * v0 ) / 2 / m_small;
    randa_big = (c * p * s_big *  v0) / 2 / m_big;
    if(is_hero)
    {
        randa = randa_big;
    }
    else
    {
        randa = randa_small;
    }
    //****************************************************
    //****************************************************
    bias_t = 0.1;  // s
    //****************************************************
}

int Trajectoryer::no_resistance_model()
{
    distance = sqrtf(pow(object_x, 2) + pow(object_z, 2));
    if(!is_solvable())
    {
        return 0;
    }
    float process_angle = atanf(object_z / object_x);
    float angle_high = (alpha + process_angle) / 2;
    float angle_low = (M_PI + process_angle - alpha) / 2;
    angle_pitch = (fabs(angle_high - now_pitch) > fabs(angle_low - now_pitch)) ? angle_low : angle_high;
    fly_t = object_x / (v0 * cos(angle_pitch));
    return 1;
}

int Trajectoryer::single_resistance_model()
{
    distance = sqrtf(pow(object_x, 2) + pow(object_z, 2));
    if(!no_resistance_model())
    {
        return 0;
    }
    float diedai_t = 0.0;
    float tan = 0.0;
    for(int i = 1; i <= 10; i++)
    {
        tan = sqrt(v0 * v0 * fly_t * fly_t - object_x * object_x);
        if(angle_pitch < 0)
        {
            tan = -tan;
        }
        diedai_t = fly_t - (ft0(fly_t, randa, tan, object_z, v0) / f_t0(fly_t, randa, tan, object_z, v0));
        fly_t = diedai_t;
        if(i == 10)
        {
            fabs(ft0(fly_t, randa, tan, object_z, v0)) > 0.01;
            RCLCPP_INFO(get_logger(), "the mistake is too large!");
            return 0;
        }
        if(fabs(ft0(fly_t, randa, tan, object_z, v0)) < 0.001)
        {
            cout << "di"  << i << "end";
            break;
        }
    }
    angle_pitch = acos(object_x / v0 / fly_t);
    return 1;
}

int Trajectoryer::single_resistance_model_two()
{
    distance = sqrtf(pow(object_x, 2) + pow(object_z, 2));
    // if(!no_resistance_model())
    // {
    //     return 0;
    // }
    float k1 = 0.019;//小弹丸的空气阻力系数
    // if(is_hero)
    // {
    //     k1 = randa * 44.5 / 1000;
    // }
    // else
    // {
    //     k1 = randa * 3.2 / 1000;
    // }
    float a_now_pitch = 0.0;
    if(now_pitch * 57.3f > 180)
    {
        a_now_pitch = (now_pitch * 57.3f - 360) / 57.3f;
    }
    else
    {
        a_now_pitch = now_pitch;
    }
    float a = (exp(k1*distance) - 1) / k1;
    float b = (g * pow(exp(k1*distance)-1,2)) / (2*pow(k1,2)*pow(v0,2));
    float delta = pow(a,2) - 4*b*(object_z+b);
    float tan_angle_1 = (a+sqrt(delta)) / (2*b);
    float tan_angle_2 = (a-sqrt(delta)) / (2*b);
    float angle_init = atan2(object_z, distance);	//rad弧度，补偿前的角度
    float angle_actual_1 = atan(tan_angle_1);
    float angle_actual_2 = atan(tan_angle_2);//rad
    // angle_pitch = (fabs(angle_actual_1 - a_now_pitch) > fabs(angle_actual_2 - a_now_pitch)) ? angle_actual_2 : angle_actual_1;//取绝对值小的那个 
    angle_pitch = angle_actual_2;
    fly_t = (float)((exp(k1 * distance) - 1) / (k1 * v0 * cos(angle_pitch)));//更新飞行时间
    return 1;
}

bool Trajectoryer::is_solvable()
{
    float process = object_z  + g * pow(object_x, 2) /  pow(v0, 2);
    if(distance >= process)
    {
        alpha = asinf(process / distance);
        return true;
    }
    RCLCPP_INFO(get_logger(), "Can not solve!");
    angle_yaw = now_yaw;
    angle_pitch = now_pitch;
    is_shoot = false;
    return false;
}

//need_init : bias_t 
//            ros_x ros_y ros_z vx vy vz v_yaw


int Trajectoryer::solve_trajectory()
{
    float need_t = fly_t + bias_t;
    float yaw_delay = need_t * v_yaw;
    float tar_yaw = yaw + yaw_delay;
    int use_1 = 1;
    int i = 0;
    int idx = 0;
    ros_x = ros_x + vx * need_t;
    ros_y = ros_y + vy * need_t;
    ros_z = ros_z + vz * need_t;
    float xiangdui_yaw = 0.0;
    if(ros_y < 0)
    {
        xiangdui_yaw = atan2(ros_y, ros_x);
    }
    else
    {
        xiangdui_yaw = atan2(ros_y, ros_x) + M_PI;
    }
    if(armor_num == 2)
    {
        for (i = 0; i<2; i++) {
        result position_result;
        float tmp_yaw = tar_yaw + i * M_PI;
        float r = r_1;
        position_result.x = ros_x - r*cos(tmp_yaw); 
        position_result.y = ros_y - r*sin(tmp_yaw);
        position_result.z = ros_z;
        position_result.yaw = tmp_yaw;
        results.push_back(position_result);
        }

        float yaw_diff_min = fabs(results.at(0).yaw - xiangdui_yaw);
        float temp_yaw_diff = fabs(results.at(1).yaw - xiangdui_yaw);
        if(temp_yaw_diff < yaw_diff_min);
        {
            yaw_diff_min = temp_yaw_diff;
            idx = i;
        }
    }
    else if(armor_num == 3)
    {
        for (i = 0; i<3; i++) 
        {
        result position_result;
        float tmp_yaw = tar_yaw + i * 2.0 * M_PI/3.0;  // 2/3PI
        float r =  (r_1 + r_2)/2;   //理论上r1=r2 这里取个平均值                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        position_result.x = ros_x - r*cos(tmp_yaw);
        position_result.y = ros_y - r*sin(tmp_yaw);
        position_result.z = ros_z;
        position_result.yaw = tmp_yaw;
        results.push_back(position_result);
        }
            // 2       1

            //     0
        float yaw_diff_min = cos(results.at(0).yaw - xiangdui_yaw);
        for(i = 1; i<3;i++)
        {
            float temp_yaw_diff = cos(results.at(i).yaw - xiangdui_yaw);
            if(temp_yaw_diff > yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
        //                            i        j

        // new idea  : to choose the       k      position to shoot
    }
    else
    {
        for (i = 0; i<4; i++) {
        result position_result;
        float tmp_yaw = tar_yaw + i * M_PI/2.0;
        float r = use_1 ? r_1 : r_2;
        position_result.x = ros_x - r*cos(tmp_yaw);
        position_result.y = ros_y - r*sin(tmp_yaw);
        position_result.z = use_1 ? ros_z : ros_z + dz;
        position_result.yaw = tmp_yaw;
        results.push_back(position_result);
        use_1 = !use_1;
        }

        float yaw_diff_min = cos(results.at(0).yaw - xiangdui_yaw);
        for(int i = 1; i<4; i++)
        {
            float temp_yaw_diff = cos(results.at(i).yaw - xiangdui_yaw);
            if(temp_yaw_diff > yaw_diff_min);
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
    }
    // object_x = results.at(idx).x + vx * need_t;
    // object_y = results.at(idx).y + vy * need_t;
    // object_z = results.at(idx).z + vz * need_t;
    object_x = ros_x + vx * need_t ;
    object_y = ros_y + vy * need_t;
    object_z = ros_z + vz * need_t;
    // if(single_resistance_model() == 0)
    // {
    //     return 0;
    // }
    single_resistance_model_two();
    if(ros_y < 0)
    {
        angle_yaw = atan2(object_y, object_x);
    }
    else
    {
        angle_yaw = atan2(object_y, object_x);
    }
    RCLCPP_INFO(get_logger(), "ros_x: %f", ros_x);
    RCLCPP_INFO(get_logger(), "ros_y: %f", ros_y);
    RCLCPP_INFO(get_logger(), "ros_z: %f", ros_z);
    RCLCPP_INFO(get_logger(), "idx: %f", idx);
    RCLCPP_INFO(get_logger(), "object_x: %f", object_x);
    RCLCPP_INFO(get_logger(), "object_y: %f", object_y);
    RCLCPP_INFO(get_logger(), "object_z: %f", object_z);
    RCLCPP_INFO(get_logger(), "angle_pitch: %f", angle_pitch);
    RCLCPP_INFO(get_logger(), "angle_yaw: %f", angle_yaw);
    RCLCPP_INFO(get_logger(), "fly_t: %f", fly_t);
    return 1;
}

void Trajectoryer::test()
{
    object_x = 10;
    object_z = 2;
    v0 = 16;
    now_pitch = 20 / 57.3f;
    now_yaw = 10 / 57.3f;
    no_resistance_model();
    cout << "distance" << distance << endl;
    cout << "angle_pitch" << angle_pitch << endl;
    cout << "fly_t" << fly_t << endl;;
    single_resistance_model();
    cout << "----------------------" << endl;
    cout << "randa_small" << randa_small << endl;
    cout << "randa_big" << randa_big << endl;
    cout << "angle_pitch" << angle_pitch  << endl;
    cout << "fly_t" << fly_t << endl;
    single_resistance_model_two();
    cout << "----------------------" << endl;
    cout << "angle_pitch" << angle_pitch  << endl;
    cout << "fly_t" << fly_t << endl;
    armor_num = 4;
    yaw = 10 / 53.7f;
    v_yaw = 0.2;
    vx = 0.4;
    ros_x = 10;
    r_1 = 0.4;
    r_2 = 0.4;
    object_x = ros_x - r_1 *cos(now_yaw);
    vy = 0.3;
    ros_y = 1;
    vz = 0.2;
    ros_z = 2;
    object_z = ros_x - r_1*cos(now_pitch);
    dz = 0.1;
    solve_trajectory();
    cout << "----------------------" << endl;
    cout << "angle_pitch" << angle_pitch  << endl;
    cout << "angle_yaw" << angle_yaw << endl;
    cout << "object_x" << object_x << endl;
    cout << "object_y" << object_y << endl;
    cout << "object_z" << object_z << endl;
    cout << "ros_x" << ros_x << endl;
    cout << "ros_y" << ros_y << endl;
    cout << "ros_z" << ros_z << endl;
    cout << "fly_t" << fly_t << endl;
    cout << "----------------------------------" << endl;
    auto_aim_interfaces::msg::Serial result;
    result.pitch = 0.6666;
    result.yaw = 0.5555;
    result_pub_->publish(result);
    RCLCPP_INFO(get_logger(), "send need angle!");
}

void Trajectoryer::target_callback(const auto_aim_interfaces::msg::Target msg)
{
    RCLCPP_INFO(get_logger(), "begin to solve!");
    is_tracking = msg.tracking;
    id = msg.id;
    armor_num = msg.armors_num;
    yaw = msg.yaw;
    v_yaw = msg.v_yaw;
    vx = msg.velocity.x;
    ros_x = msg.position.x;
    vy = msg.velocity.y;
    ros_y = msg.position.y + y_bias;
    vz = msg.velocity.z;
    ros_z = msg.position.z + z_bias;
    r_1 = msg.radius_1;
    r_2 = msg.radius_2;
    dz = msg.dz;
    RCLCPP_INFO(get_logger(), "ros_x: %f", ros_x);
    RCLCPP_INFO(get_logger(), "ros_y: %f", ros_y);
    RCLCPP_INFO(get_logger(), "ros_z: %f", ros_z);
    RCLCPP_INFO(get_logger(), "vx: %f", vx);
    RCLCPP_INFO(get_logger(), "vy: %f", vy);
    RCLCPP_INFO(get_logger(), "vz: %f", vz);
    RCLCPP_INFO(get_logger(), "yaw: %f", yaw);
    RCLCPP_INFO(get_logger(), "v_yaw: %f", v_yaw);
    RCLCPP_INFO(get_logger(), "r_1: %f", r_1);
    RCLCPP_INFO(get_logger(), "r_2: %f", r_2);
    RCLCPP_INFO(get_logger(), "dz: %f", dz);
    RCLCPP_INFO(get_logger(), "armor_num: %d", armor_num);
    RCLCPP_INFO(get_logger(), "is_tracking: %d", is_tracking);
    RCLCPP_INFO(get_logger(), "id: %d", id);
    if(is_tracking)
    {
        auto_aim_interfaces::msg::Serial result;
        if(solve_trajectory() == 1)
        {
            result.is_shoot = true;
        }
        RCLCPP_INFO(get_logger(), "need_pitch: %f", angle_pitch);
        RCLCPP_INFO(get_logger(), "need_yaw: %f", angle_yaw);
        // if(is_shoot)
        // {
            float xianzhi_pitch = 0.0;
            float xianzhi_yaw = 0.0;
            xianzhi_pitch = angle_pitch * 57.3f;
            // if(xianzhi_pitch < 160 || xianzhi_pitch > 190)
            // {
            //     xianzhi_pitch = 170;
            // }
            xianzhi_yaw = angle_yaw * 57.3f;
            result.pitch = xianzhi_pitch;
            result.yaw = xianzhi_yaw;
            result.is_tracking = 1;
            result_pub_->publish(result);
            RCLCPP_INFO(get_logger(), "send need angle!");
            is_shoot = false;
        //}
    }
    else
    {
        auto_aim_interfaces::msg::Serial result;
        result.pitch = 0.0;
        result.yaw = 0.0;
        result.is_tracking = false;
        result_pub_->publish(result);
    }
}

void Trajectoryer::angle_callback(const auto_aim_interfaces::msg::Serial msg)
{
    now_pitch = msg.pitch / 57.3f;
    now_yaw = msg.yaw / 57.3f;
    is_shoot = msg.is_shoot;
    RCLCPP_INFO(get_logger(), "now_pitch: %f", now_pitch);
    RCLCPP_INFO(get_logger(), "now_yaw: %f", now_yaw);
    visualization_msgs::msg::Marker aiming_point_;
    if (abs(object_x) > 0.01) {
        aiming_point_.header.stamp = this->now();
        aiming_point_.pose.position.x = object_x;
        aiming_point_.pose.position.y = object_y;
        aiming_point_.pose.position.z = object_z;
        maker_pub_->publish(aiming_point_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Trajectoryer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// #include "rclcpp_components/register_node_macro.hpp"

// // Register the component with class_loader.
// // This acts as a sort of entry point, allowing the component to be discoverable when its library
// // is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(Trajectoryer)
