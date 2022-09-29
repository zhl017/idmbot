#include "../include/idmbot_libs/idmbot_arm.h"

idmbot_arm::idmbot_arm()
    :   torque_state_(false)
{
}

idmbot_arm::~idmbot_arm()
{
    colseDynamixel();
}

void idmbot_arm::colseDynamixel(void)
{
    setTorque(false);
}

bool idmbot_arm::init()
{
    dxl_.init(DEVICENAME, BAUDRATE);
  
    for (uint8_t num = 0; num < JOINT_CNT; num++)
    {
        dxl_.ping(joint_id_[num]);
        dxl_.torqueOff(joint_id_[num]);
        dxl_.setOperatingMode(joint_id_[num], 3);
        dxl_.itemWrite(joint_id_[num], "CW_Angle_Limit", joint_pos_limit[num][0]);
        dxl_.itemWrite(joint_id_[num], "CCW_Angle_Limit", joint_pos_limit[num][1]);
        dxl_.torqueOn(joint_id_[num]);
    }

    for (uint8_t num = 0; num < GRIPPER_CNT; num++)
    {
        dxl_.ping(gripper_id_[num]);
        dxl_.torqueOff(gripper_id_[num]);
        dxl_.setOperatingMode(gripper_id_[num], 3);
        dxl_.itemWrite(gripper_id_[num], "CW_Angle_Limit", gripper_pos_limit[num][0]);
        dxl_.itemWrite(gripper_id_[num], "CCW_Angle_Limit", gripper_pos_limit[num][1]);
        dxl_.torqueOn(gripper_id_[num]);
    }

    double joint_velocity[JOINT_CNT] = {0.8};
    double joint_position[JOINT_CNT] = {1.5};
    double gripper_velocity[GRIPPER_CNT] = {0.8, 0.8};
    double gripper_position[GRIPPER_CNT] = {0.0, 0.0};

    setJointVelocity(joint_velocity);
    setJointPosition(joint_position);
    setGripperVelocity(gripper_velocity);
    setGripperPosition(gripper_position);

    torque_state_ = true;

    return true;
}

bool idmbot_arm::setTorque(bool onoff)
{
    bool result = false;

    for (uint8_t num = 0; num < JOINT_CNT; num++)
    {
        result = dxl_.torque(joint_id_[num], onoff);
    }

    for (uint8_t num = 0; num < GRIPPER_CNT; num++)
    {
        result = dxl_.torque(gripper_id_[num], onoff);
    }

    if ( result == true ) torque_state_ = onoff;

    return result;
}

bool idmbot_arm::setJointPosition(double *set_data)
{
    int32_t goal_position[JOINT_CNT];

    for (uint8_t num = 0; num < JOINT_CNT; num++)
    {
        goal_position[num] = dxl_.convertRadian2Value(joint_id_[num], set_data[num]);
        dxl_.goalPosition(joint_id_[num], goal_position[num]);
    }

    return true;
}

bool idmbot_arm::setJointVelocity(double *set_data)
{
    int32_t goal_velocity[JOINT_CNT];

    for (uint8_t num = 0; num < JOINT_CNT; num++)
    {
        goal_velocity[num] = dxl_.convertVelocity2Value(joint_id_[num], set_data[num]);
        dxl_.goalVelocity(joint_id_[num], goal_velocity[num]);
    }

    return true;
}

bool idmbot_arm::setGripperPosition(double *set_data)
{
    int32_t goal_position[GRIPPER_CNT];

    for (uint8_t num = 0; num < GRIPPER_CNT; num++)
    {
        goal_position[num] = dxl_.convertRadian2Value(gripper_id_[num], set_data[num]);
        dxl_.goalPosition(gripper_id_[num], goal_position[num]);
    }

    return true;
}

bool idmbot_arm::setGripperVelocity(double *set_data)
{
    int32_t goal_velocity[GRIPPER_CNT];

    for (uint8_t num = 0; num < GRIPPER_CNT; num++)
    {
        goal_velocity[num] = dxl_.convertVelocity2Value(gripper_id_[num], set_data[num]);
        dxl_.goalVelocity(gripper_id_[num], goal_velocity[num]);
    }

    return true;
}

bool idmbot_arm::getPosition(double *get_data)
{
    float joint_present_position[JOINT_CNT];
    float gripper_present_position[GRIPPER_CNT];

    for (uint8_t num = 0; num < JOINT_CNT; num++)
    {
        dxl_.getRadian(joint_id_[num], &joint_present_position[num]);
        get_data[num] = joint_present_position[num];
    }

    for (uint8_t num = 0; num < GRIPPER_CNT; num++)
    {
        dxl_.getRadian(gripper_id_[num], &gripper_present_position[num]);
        get_data[JOINT_CNT + num] = gripper_present_position[num];
    }

    return true;
}

bool idmbot_arm::getVelocity(double *get_data)
{
    int32_t joint_present_velocity[JOINT_CNT];
    int32_t gripper_present_velocity[GRIPPER_CNT];

    for (uint8_t num = 0; num < JOINT_CNT; num++)
    {
        dxl_.itemRead(joint_id_[num], "Present_Speed", &joint_present_velocity[num]);
        get_data[num] = dxl_.convertValue2Velocity(joint_id_[num], joint_present_velocity[num]);
    }
    
    for (uint8_t num = 0; num < GRIPPER_CNT; num++)
    {
        dxl_.itemRead(gripper_id_[num], "Present_Speed", &gripper_present_velocity[num]);
        get_data[JOINT_CNT + num] = dxl_.convertValue2Velocity(gripper_id_[num], gripper_present_velocity[num]);
    }

    return true;
}

bool idmbot_arm::getTorqueState(void)
{
    return torque_state_;
}