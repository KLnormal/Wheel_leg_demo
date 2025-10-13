//
// Created by 15082 on 2025/10/2.
//

#ifndef APP_LEG_MOTOR_H
#define APP_LEG_MOTOR_H

#include "dev_motor_dji.h"
#include "dev_motor_dm.h"

#include <fast_math_functions.h>


class joint {
public:
    joint();
    joint(const char *name, const Motor::DMMotor::Model &model, const Motor::DMMotor::Param &param,int dir, float zero, float max_tor)
    :_m(name, model,param),_zero(zero),_dir(dir),_max_tor(max_tor) {
    }
    void joint_enable();
    void joint_disable() {
        _m.disable();
    }
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
/*
 * 铁损阻力：72rpm/Nm
 * 减速比：3591/187
 * 扭矩常数：0.3Nm/A
 * 机械转角：0~8192
 * 转子转速单位：RPM
 * 电流： -20A~20A  -16384~16384
 */
#define ENCODER_RANGE 8192
#define CURRENT_RANGE 16384
#define CURRENT_CAST(current) (current/20*CURRENT_RANGE)
#define TOR_CAST(tor) CURRENT_CAST(tor/0.3)

class dynamic_motor {
    public:
    dynamic_motor();
    dynamic_motor(const char *name,const Motor::DJIMotor::Model &model,const Motor::DJIMotor::Param &param, int16_t dir)
        : _motor(name,model,param), _dir(dir) {
    }
    void motor_init() {_motor.init();}
    void motor_tor(float tor) {
        int16_t ctrl_current = (int16_t)TOR_CAST(tor);
        _motor.update(ctrl_current*_dir);
        motor_deg_clc();
    }
    float get_deg() const {return (float)_dir*_deg;}
    float get_speed() const {return (float)_dir*_speed;}
    int32_t _rounds = 0;
private:
    void motor_deg_clc();
    Motor::DJIMotor _motor;
    int16_t encoder = 0, _dir = 0;
    float _deg = 0;
    float _dis = 0;
    float _speed = 0;
};

extern joint joint1;
extern joint joint2;
#endif //APP_LEG_MOTOR_H
