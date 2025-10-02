//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"
#include "app_wheel_leg_datalist.h"
#include "app_leg_forward.h"
#include "app_leg_motor.h"
#include "sys_task.h"

#ifdef __cplusplus
extern "C" {
#endif


namespace Chassis_Wheel_Leg {
class leg {
public:
    leg();
    leg(joint *joint1, joint *joint2, dynamic_motor *motor) :_joint1(joint1), _joint2(joint2), _motor(motor) {
    };
    void leg_init() {
        _joint1->joint_enable();
        OS::Task::SleepMilliseconds(1);
        _joint2->joint_enable();
    }
    void leg_force_ctrl(float force_x, float force_y);
    void joint_get_polar();
    void leg_ctrl(float tor1, float tor2);
    float _phi1,_phi4;
    float _L0, _Phi0;
    float _x0,_y0;
private:
    joint *_joint1,*_joint2;
    dynamic_motor *_motor;
    float tor_j1, tor_j2, tor_motor;
    float distance;
};
}


void app_chassis_init();
void app_chassis_task(void *argument);



#ifdef __cplusplus
}
#endif