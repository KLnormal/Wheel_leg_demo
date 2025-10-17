//
// Created by 15082 on 2025/10/2.
//

#include "app_leg_motor.h"

#include "bsp_uart.h"
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

void dynamic_motor::motor_deg_clc() {
    int16_t temp_encoder = _motor.feedback_.angle;
    // bsp_uart_printf(E_UART_DEBUG, "%d\n", abs(temp_encoder - _motor.an));
    if(abs(temp_encoder - encoder) > ENCODER_RANGE/2) {
        (temp_encoder - encoder)>0?_rounds--:_rounds++;
    }
    encoder = temp_encoder;
    _deg = (float)_rounds*2*PI + (float)encoder/ENCODER_RANGE*2*PI;
    _speed = _motor.feedback_.speed;
}