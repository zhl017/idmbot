#include "../include/idmbot_libs/idmbot_wheel.h"

idmbot_wheel::idmbot_wheel()
    :   torque_state_(false),
        limit_max_velocity_(LIMIT_MAX_VELOCITY)
{
}

idmbot_wheel::~idmbot_wheel()
{
    closeDynamixel();
}

void idmbot_wheel::closeDynamixel(void)
{
    setTorque(false);
}

bool idmbot_wheel::init()
{
    dxl_.init(DEVICENAME, BAUDRATE);
  
    for (uint8_t num = 0; num < WHEEL_CNT; num++)
    {
        dxl_.ping(wheel_id_[num]);
        dxl_.torqueOff(wheel_id_[num]);
        dxl_.setOperatingMode(wheel_id_[num], 1);
        dxl_.torqueOn(wheel_id_[num]);
    }

    torque_state_ = true;

    return true;
}

bool idmbot_wheel::setTorque(bool onoff)
{
    bool result = false;

    for (uint8_t num = 0; num < WHEEL_CNT; num++)
    {
        result = dxl_.torque(wheel_id_[num], onoff);
    }
    
    if (result = true) torque_state_ = onoff;
    
    return result;
}


bool idmbot_wheel::setVelocity(float *set_data)
{
    int32_t goal_velocity[WHEEL_CNT];

    for(uint8_t num = 0; num < WHEEL_CNT; num++)
    {
        goal_velocity[num] = set_data[num];
        dxl_.goalVelocity(wheel_id_[num], goal_velocity[num]);
    }

    return true;
}

bool idmbot_wheel::controlMotor(const float wheel_radius, const float wheel_separation, float *vlaue)
{
    bool result = false;
    float wheel_velocity_cmd[WHEEL_CNT];
    float lin_vel = vlaue[0];
    float ang_vel = vlaue[1];

    wheel_velocity_cmd[0] = lin_vel - (ang_vel * wheel_separation / 2);
    wheel_velocity_cmd[1] = lin_vel + (ang_vel * wheel_separation / 2);

    wheel_velocity_cmd[0] = constrain(wheel_velocity_cmd[0] * VELOCITY_CONSTANT_VALUE / wheel_radius, -limit_max_velocity_, limit_max_velocity_);
    wheel_velocity_cmd[1] = constrain(wheel_velocity_cmd[1] * VELOCITY_CONSTANT_VALUE / wheel_radius, -limit_max_velocity_, limit_max_velocity_);

    result = setVelocity(wheel_velocity_cmd);

    if (result == false) return false;
    
    return true;
}

bool idmbot_wheel::readEncoder(double *get_data)
{
    int32_t present_position[WHEEL_CNT];

    for(uint8_t num = 0; num < WHEEL_CNT; num++)
    {
        dxl_.getPresentPositionData(wheel_id_[num], &present_position[num]);
        get_data[num] = present_position[num];
    }

    return true;
}

bool idmbot_wheel::getTorqueState(void)
{
    return torque_state_;
}

double idmbot_wheel::getVoltage(void)
{
    int32_t data = 0;

    dxl_.itemRead(LEFT_ID, "Present_Input_Voltage", &data);

    return ((double)data * 0.1);
}

float idmbot_wheel::constrain(float x, float a, float b)
{
    if ( x < a ) return a;
    else if ( b < x ) return b;
    else return x;
}
