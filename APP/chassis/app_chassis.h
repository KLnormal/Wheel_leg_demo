//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"
#include "app_wheel_leg_datalist.h"
#include "app_leg_forward.h"
#include "app_leg_motor.h"

#ifdef __cplusplus
extern "C" {
#endif


namespace Chassis_Wheel_Leg {
class leg {
public:
    leg();
    leg(joint &joint1, joint &joint2) :_joint1(joint1), _joint2(joint2) {
    }
    void leg_init() {
        joint1.joint_enable();
        joint2.joint_enable();
    }
    void leg_force_ctrl(float force_x, float force_y);
    void joint_get_polar() {

    }
private:
    void leg_ctrl(float tor1, float tor2);
    joint _joint1,_joint2;
};
}


void app_chassis_init();
void app_chassis_task(void *argument);



#ifdef __cplusplus
}
#endif