#ifndef IDMBOT_ARM_H_
#define IDMBOT_ARM_H_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define DEVICENAME     "/dev/ttyUSB0"
#define BAUDRATE        1000000

#define JOINT_1         1
#define JOINT_2         2
#define JOINT_3         3
#define JOINT_4         4
#define JOINT_CNT       4

#define GRIPPER_ID      5
#define GRIPPER_CNT     1

class idmbot_arm
{
private:
    DynamixelWorkbench dxl_;

    bool torque_state_;
    uint8_t joint_id_[JOINT_CNT] = {JOINT_1, JOINT_2, JOINT_3, JOINT_4};
    uint8_t gripper_id_[GRIPPER_CNT] = {GRIPPER_ID};

public:
    idmbot_arm();
    ~idmbot_arm();

    void colseDynamixel(void);
    bool init();

    bool setTorque(bool onoff);
    bool setJointPosition(double *set_data);
    bool setJointVelocity(double *set_data);
    bool setGripperPosition(double *set_data);
    bool setGripperVelocity(double *set_data);

    bool getPosition(double *get_data);
    bool getVelocity(double *get_data);

    bool getTorqueState(void);
};

#endif