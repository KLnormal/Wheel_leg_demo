//
// Created by 15082 on 2025/10/2.
//

#ifndef APP_LEG_MOTOR_H
#define APP_LEG_MOTOR_H

#include "dev_motor_dm.h"

void leg_init();
float* leg_deg();
namespace Leg {
class joint {
public:
    joint();
    joint(const char *name, const Motor::DMMotor::Model &model, const Motor::DMMotor::Param &param,int dir, float zero, float max_tor)
    :_m(name, model,param),_zero(zero),_dir(dir),_max_tor(max_tor) {
    }
    void joint_enable() {
        _m.init();
        _m.enable();
    }
    float deg_clc() {
        float temp_deg = _m.feedback_.pos;
        if(_dir == 1 && _zero == 0) return temp_deg;
        else {
            temp_deg = (temp_deg-_zero)*(float)_dir;
            return temp_deg;
        }
    }
    void joint_ctrl(float tor) {
        _m.update(tor);
    }
private:
    Motor::DMMotor _m;
    float _zero;
    int _dir;
    float _max_tor;
};
}

extern Leg::joint joint1;
extern Leg::joint joint2;

#endif //APP_LEG_MOTOR_H
