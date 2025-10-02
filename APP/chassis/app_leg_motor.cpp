//
// Created by 15082 on 2025/10/2.
//

#include "app_leg_motor.h"

#include "sys_task.h"

#include <numbers>

float joint::deg_clc() {
    float temp_deg = _m.status.pos;
    if(_dir == 1 && _zero == 0) {
        joint_deg = temp_deg;
        return temp_deg;
    }
    else {
        temp_deg = (temp_deg-_zero)*(float)_dir;
        joint_deg = temp_deg;
        return temp_deg;
    }
}
void joint::joint_enable() {
    _m.init();
    _m.enable();
}