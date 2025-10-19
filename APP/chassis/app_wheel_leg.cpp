//
// Created by 15082 on 2025/10/19.
//

#include "app_wheel_leg.h"

#include <ios>
#include <bits/codecvt.h>
#include <bits/ios_base.h>

float total::wheel_leg::leg_length_ctrl(float length, leg_direction leg_switch) {
return 0;
}
float total::wheel_leg::PID(float target, float current, float Kp, float Ki, float Kd, float I_limit, float sum_limit, leg_direction leg_switch) {
    simple_PID *temp_PID = nullptr;
    if(leg_switch == Left_leg) temp_PID = &left_length_PD;
    else temp_PID = &right_length_PD;

    temp_PID->I_limit = I_limit;
    temp_PID->Sum_limit = sum_limit;
    temp_PID->Kp = Kp;
    temp_PID->Ki = Ki;
    temp_PID->Kd = Kd;

    temp_PID->old = temp_PID->current;
    temp_PID->current = current;
    temp_PID->old_error = temp_PID->error;
    temp_PID->error = target - current;
    temp_PID->temp_p = temp_PID->error*temp_PID->Kp;
    temp_PID->temp_d = (temp_PID->error - temp_PID->old_error)*temp_PID->Kd/0.001f;
    temp_PID->temp_i += temp_PID->error*temp_PID->Ki * 0.001f;
    if(temp_PID->temp_i > temp_PID->I_limit || temp_PID->temp_i < -temp_PID->I_limit) temp_PID->temp_i = temp_PID->temp_i/abs(temp_PID->temp_i)*temp_PID->I_limit;
    temp_PID->sum = temp_PID->temp_d+temp_PID->temp_i+temp_PID->temp_p;
    if(temp_PID->sum > temp_PID->Sum_limit || temp_PID->sum < -temp_PID->Sum_limit) temp_PID->sum = temp_PID->sum/abs(temp_PID->sum)*temp_PID->Sum_limit;
return temp_PID->sum;
}
