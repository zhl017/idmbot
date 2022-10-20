#ifndef IDMBOT_WHEEL_H_
#define IDMBOT_WHEEL_H_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"

#define LIMIT_MAX_VELOCITY          200

#define LEFT_ID                     6
#define RIGHT_ID                    7

#define WHEEL_CNT                   2

#define VELOCITY_CONSTANT_VALUE     41.69988758
/*
    V = r * w = r * (           RPM           * 0.10472 )
              = r * ( 0.229 * Goal_Velocity ) * 0.10472

    Goal_Velocity = V / r * 41.6988758
*/

class idmbot_wheel
{
private:
    DynamixelWorkbench dxl_;

    bool torque_state_;
    uint8_t wheel_id_[WHEEL_CNT] = {LEFT_ID, RIGHT_ID};
    uint16_t limit_max_velocity_;

    float constrain(float x, float a, float b);

public:
    idmbot_wheel();
    ~idmbot_wheel();

    void closeDynamixel(void);
    bool init();
    bool setTorque(bool onoff);
    bool setVelocity(float *set_data);

    bool controlMotor(const float wheel_radius, const float wheel_separation, float *vlaue);

    bool readEncoder(double *get_data);
    bool getTorqueState(void);
    double getVoltage(void);
};

#endif