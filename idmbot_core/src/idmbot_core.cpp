// #include "idmbot_core/idmbot_core_config.h"
#include "../include/idmbot_core/idmbot_core_config.h"

idmbot_core::idmbot_core(){}

idmbot_core::~idmbot_core(){}

void idmbot_core::process(void)
{
    //pub
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    dxl_voltage_pub = nh.advertise<std_msgs::Float64>("voltage", 10);

    //sub
    cmd_vel_sub = nh.subscribe("cmd_vel", 10, &idmbot_core::cb_cmd_vel, this);
    dxl_power_sub = nh.subscribe("dxl_power", 10, &idmbot_core::cb_dxl_power, this);
    reset_sub = nh.subscribe("reset", 10, &idmbot_core::cb_reset, this);

    wheel_driver.init();
    // arm_driver.init();

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    odom_time = ros::Time::now();

    if (initOdom() == true) ROS_INFO("Init Odom.");
    if (initJointstates() == true) ROS_INFO("Init JointStates.");

    ros::Rate r = 10;

    

    while(ros::ok())
    {
        current_time = ros::Time::now();
        double dt = ( current_time - last_time ).toNSec();
        if (dt >= 1000/CONTROL_MOTOR_SPEED_FREQUENCY)
        {
            // ROS_INFO("speed_control");
            wheel_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
            tTime[0] = current_time;
        }
        if (dt >= 1000/JOINT_CONTROL_FREQUENCY)
        {
            // ROS_INFO("joint_control");
            tTime[1] = current_time;
        }
        if (dt >= 1000/DRIVE_INFORMATION_PUBLISH_FREQUENCY)
        {
            // ROS_INFO("info_pub");
            updateInfo();
            tTime[2] = current_time;
        }
        last_time = current_time;

        ros::spinOnce();
        r.sleep();
    }
}

void idmbot_core::cb_cmd_vel(const geometry_msgs::Twist &msg)
{
    goal_velocity[LINEAR] = msg.linear.x;
    goal_velocity[ANGULAR] = msg.angular.z;
}

void idmbot_core::cb_reset(const std_msgs::Empty &msg)
{
    init_odom = false;
    initOdom();
    init_encoder = false;
}

void idmbot_core::cb_dxl_power(const std_msgs::Bool &msg)
{
    bool dxl_power = msg.data;

    wheel_driver.setTorque(dxl_power);
    // arm_driver.setTorque(dxl_power);
}

void idmbot_core::updateInfo(void)
{
    ros::Time stamp_now = ros::Time::now();

    double dt = (stamp_now - odom_time).toNSec();
    odom_time = stamp_now;

    calcOdom(dt * 0.001);
    updateOdom();
    odom.header.stamp = stamp_now;
    odom_pub.publish(odom);

    updateJointstates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(joint_states);

    updateTF();
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    updateVoltage();
    dxl_voltage_pub.publish(voltage);
}

bool idmbot_core::initOdom(void)
{
    if ( init_odom == false )
    {
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint"; 

        for (int index = 0; index < 3; index++)
        {
            odom_pose[index] = 0.0;
            odom_vel[index]  = 0.0;
        }

        odom.pose.pose.position.x = 0.0;
        odom.pose.pose.position.y = 0.0;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation.w = 0.0;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.angular.z = 0.0;

        init_odom = true;
    }

    return true;
}

void idmbot_core::updateOdom(void)
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_pose[2]);

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];
}

void idmbot_core::calcOdom(double diff_time)
{
    float* orientation;
    double wheel_l, wheel_r;      // rotation value of wheel [rad]
    double theta_l, theta_r;
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v = w = 0.0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
        return;

    wheel_l = TICK2RAD * (double) last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double) last_diff_tick[RIGHT];
    theta_l = last_rad[LEFT];
    theta_r = last_rad[RIGHT];

    if (isnan(wheel_l))
        wheel_l = 0.0;

    if (isnan(wheel_r))
        wheel_r = 0.0;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    theta = WHEEL_RADIUS * (theta_r - theta_l) / WHEEL_SEPARATION;  
    // orientation = sensors.getOrientation();
    // theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
    //                 0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

    delta_theta = theta - last_theta;

    // compute odometric pose
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;
}

bool idmbot_core::initJointstates(void)
{
    joint_states.header.frame_id = "base_link";

    joint_states.name = {"wheel_left", "wheel_right"};

    return true;
}

void idmbot_core::updateJointstates(void)
{
    updateMotor();
    joint_states.position = {last_rad[LEFT], last_rad[RIGHT]};
    joint_states.velocity = {last_velocity[LEFT], last_velocity[RIGHT]};
}

void idmbot_core::updateMotor(void)
{
    int32_t current_tick = 0;
    static int32_t last_tick[WHEEL_NUM] = {0,0};
    double encoder_tick[WHEEL_NUM];

    wheel_driver.readEncoder(encoder_tick);

    if (init_encoder == false)
    {
        for (int index = 0; index < WHEEL_NUM; index++)
        {
            last_diff_tick[index] = 0.0;
            last_tick[index]      = 0;
            last_rad[index]       = 0.0;
            last_velocity[index]  = 0.0;
        }  

        last_tick[LEFT] = encoder_tick[LEFT];
        last_tick[RIGHT] = encoder_tick[RIGHT];

        init_encoder = true;
        return;
    }

    current_tick = encoder_tick[LEFT];

    last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
    last_tick[LEFT] = current_tick;
    last_rad[LEFT] += TICK2RAD * (double) last_diff_tick[LEFT];

    current_tick = encoder_tick[RIGHT];

    last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
    last_tick[RIGHT] = current_tick;
    last_rad[RIGHT] += TICK2RAD * (double) last_diff_tick[RIGHT];
}

void idmbot_core::updateTF()
{
    odom_tf.header.frame_id = odom.header.frame_id;
    odom_tf.child_frame_id = odom.child_frame_id;

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;

    odom_tf.transform.rotation = odom.pose.pose.orientation;
}

void idmbot_core::updateVoltage()
{
    voltage.data = wheel_driver.getVoltage();
    if ( voltage.data < 10 )
    {
        ROS_INFO("Voltage [ LOW ].");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "idmbot_core");
    idmbot_core idmbot;
    idmbot.process();
    return 0;
}