#ifndef IDMBOT_CORE_CONFIG_H_
#define IDMBOT_CORE_CONFIG_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "idmbot_libs/idmbot.h"

#include <math.h>

#define WHEEL_RADIUS            0.033           // 車輪半徑
#define WHEEL_SEPARATION        0.160           // 輪距
#define TURNING_RADIUS          0.153           // 旋轉半徑
#define ROBOT_RADIUS            0.153           // 車身半徑

#define MAX_LINEAR_VELOCITY     ( WHEEL_RADIUS * 2 * 3.14159265359 * 46 / 60 ) // RPM = 0.15 m/s 
#define MAX_ANGULAR_VELOCITY    ( MAX_LINEAR_VELOCITY / TURNING_RADIUS)        // VEL = 0.98 rad/s 

#define MIN_LINEAR_VELOCITY     -MAX_LINEAR_VELOCITY
#define MIN_ANGULAR_VELOCITY    -MAX_ANGULAR_VELOCITY

#define CONTROL_MOTOR_SPEED_FREQUENCY           30      // hz
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY     30      // hz
#define JOINT_CONTROL_FREQUENCY                 100     // hz

#define WHEEL_NUM                               2
#define JOINT_CNT                               1
#define GRIPPER_CNT                             2

#define LEFT                                    0
#define RIGHT                                   1

#define LINEAR                                  0
#define ANGULAR                                 1

#define DEG2RAD(x)                              ( x * 0.01745329252 )   // PI/180
#define RAD2DEG(x)                              ( x * 57.2957795131 )   // 180/PI

#define TICK2RAD                                0.001533981             // 0.087890625[deg] * PI / 180

#define TEST_DISTANCE                           0.3                     // meter
#define TEST_RADIAN                             3.14                    // 3.14 deg

class idmbot_core
{
private:
    ros::NodeHandle nh;

    // ROS Publisher
    ros::Publisher odom_pub;
    ros::Publisher joint_states_pub;
    ros::Publisher dxl_voltage_pub;

    // ROS Subscriber & Callback Function
    void cb_cmd_vel(const geometry_msgs::Twist &msg);
    float goal_velocity[WHEEL_NUM] = {0, 0};
    ros::Subscriber cmd_vel_sub;

    void cb_dxl_power(const std_msgs::Bool &msg);
    ros::Subscriber dxl_power_sub;

    void cb_reset(const std_msgs::Empty &msg);
    ros::Subscriber reset_sub;

    void cb_joint_velocity(const std_msgs::Float64MultiArray &msg);
    ros::Subscriber joint_velocity_sub;
    void cb_joint_position(const std_msgs::Float64MultiArray &msg);
    ros::Subscriber joint_position_sub;

    void cb_gripper_velocity(const std_msgs::Float64MultiArray &msg);
    ros::Subscriber gripper_velocity_sub;
    void cb_gripper_position(const std_msgs::Float64MultiArray &msg);
    ros::Subscriber gripper_positoin_sub;

    // Function
    void updateInfo(void);

    // Odom
    nav_msgs::Odometry odom;
    bool init_odom = false;
    float odom_pose[3];
    double odom_vel[3];
    bool initOdom(void);
    void updateOdom(void);
    void calcOdom(double diff_time);

    // Joint states
    sensor_msgs::JointState joint_states;
    bool initJointstates(void);
    void updateJointstates(void);

    // TF
    tf::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped odom_tf;
    void updateTF(void);

    // DYNAMIXEL 
    bool init_encoder = false;
    int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
    double last_rad[WHEEL_NUM]        = {0.0, 0.0};
    double last_velocity[WHEEL_NUM]   = {0.0, 0.0};
    void updateMotor(void);

    // Time
    ros::Time current_time, last_time, odom_time;
    ros::Time tTime[10];

    // idmbot driver
    idmbot_arm arm_driver;
    idmbot_wheel wheel_driver;

    // Voltage
    std_msgs::Float64 voltage;
    void updateVoltage(void);

public:
    idmbot_core();
    ~idmbot_core();
    void process(void);
};

#endif