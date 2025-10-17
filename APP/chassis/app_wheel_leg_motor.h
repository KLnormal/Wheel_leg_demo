//
// Created by 15082 on 2025/10/17.
//

#ifndef APP_WHEEL_LEG_MOTOR_H
#define APP_WHEEL_LEG_MOTOR_H

#include "dev_motor_dji.h"
#include "dev_motor_dm.h"

#include <arm_math_types.h>

namespace wheel_leg_motor {
class joint {
    public:
    joint();
    joint(Motor::DMMotor *joint_motor,float32_t zero, float32_t dir):joint_(joint_motor), zero_(zero), dir_(dir) {

    }
    void joint_init();
    void joint_ctrl(float tor);
    void joint_ctrl(float P, float I, float D, float sum_limit, float I_limit, float target_pos);//重载了位置PID和扭矩控制
    Motor::DMMotor *joint_;
    float32_t zero_, dir_, joint_deg_, joint_v_;
    float32_t old_pos_ = 0, old_err_ = 0, out_sum_ = 0, out_i_ = 0, out_p_ = 0, out_d_ = 0;
private:
    void joint_deg_clc();

};
class dynamic {
public:
    dynamic();
    dynamic(Motor::DJIMotor *dynamic_motor, float32_t dir);
    void motor_init();
    void motor_deg_clc();
    void tor_ctrl(float32_t tor);
    Motor::DJIMotor *dynamic_motor_;
    float32_t dir_;
    float32_t old_deg = -1000, single_deg = 0, round_cnt = 1, total_deg = 0;
private:

};
}

#endif //APP_WHEEL_LEG_MOTOR_H
