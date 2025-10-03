//
// Created by 15082 on 2025/10/2.
//

#ifndef APP_LEG_MOTOR_H
#define APP_LEG_MOTOR_H

#include "dev_motor_dji.h"
#include "dev_motor_dm.h"


class joint {
public:
    joint();
    joint(const char *name, const Motor::DMMotor::Model &model, const Motor::DMMotor::Param &param,int dir, float zero, float max_tor)
    :_m(name, model,param),_zero(zero),_dir(dir),_max_tor(max_tor) {
    }
    void joint_enable();
    float deg_clc();
    void joint_ctrl(float tor) {
        _m.control(0,0,0,0,tor*(float)_dir);
    }
private:
    Motor::DMMotor _m;
    float _zero;
    int _dir;
    float _max_tor;
    float joint_deg;
};

class dynamic_motor {
    public:
    dynamic_motor();
    dynamic_motor(char a):_a(a){}
private:
    Motor::DJIMotor _m;
    char _a;
};

extern joint joint1;
extern joint joint2;

#endif //APP_LEG_MOTOR_H
