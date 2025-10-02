//
// Created by 15082 on 2025/10/2.
//

#include "app_leg_motor.h"

#include "sys_task.h"

#include <numbers>

joint joint1("joint1",Motor::DMMotor::J4310,{
        .slave_id = 0x21,
        .master_id = 0x11,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },1,-std::numbers::pi/2,2);
joint joint2("joint2",Motor::DMMotor::J4310,{
        .slave_id = 0x22,
        .master_id = 0x12,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },1,-std::numbers::pi/2,2);

float joint_deg[4] = {};

void leg_init() {
    joint1.joint_enable();
    OS::Task::SleepMilliseconds(1);
    joint2.joint_enable();
}

float* leg_deg() {
    joint_deg[0] = joint1.deg_clc();
    joint_deg[1] = joint2.deg_clc();
    return joint_deg;
}

float* leg_ctrl(float *tor_ary) {
    joint1.joint_ctrl(tor_ary[0]);
    joint2.joint_ctrl(tor_ary[1]);
    return leg_deg();
}