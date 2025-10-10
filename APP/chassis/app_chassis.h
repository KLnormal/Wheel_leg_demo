//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"
#include "app_ins.h"
#include "app_wheel_leg_datalist.h"
#include "app_leg_motor.h"
#include "sys_task.h"

#include <matrix.h>


#ifdef __cplusplus

struct app_ins_data_t;
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
        _joint2->joint_enable();
        _motor->motor_init();
    }
    void leg_force_ctrl(float force, float tor, float dynamic_tor);
    void joint_get_polar();
    float dynamic_get_deg(){return _motor->get_deg();}
    float dynamic_get_speed() {return _motor->get_speed();}
    void leg_ctrl(float tor1, float tor2);
    float _L0, _Phi0;
    float _x0,_y0;
    float vmc_tor1,vmc_tor2;
    float _phi1,_phi4;
private:
    joint *_joint1,*_joint2;
    dynamic_motor *_motor;
    float tor_j1, tor_j2, tor_motor;
    float distance;
    float _phi2,_phi3;
    float _xb,_xd,_yb,_yd;
};

struct leg_info{
     float phi, dot_phi;
     float theta, dot_theta;
     float dis_x, dot_dis_x;
};

class chassis {
public:
    chassis();
    chassis(leg *left_leg, leg *right_leg, float32_t *K, const app_ins_data_t* ins, float R)
    : _left_leg(left_leg), _right_leg(right_leg), _ins(ins), _wheel_R(R) {
        memcpy(_data_k,&K,sizeof(float32_t)*12);
    };
    void chassis_init() {
        _left_leg->leg_init();
        _right_leg->leg_init();
    }
    void chassis_clc(leg *leg, leg_info *leg_struct) {
        leg_struct->phi = -_ins->roll;
        leg_struct->dot_phi = -_ins->raw.gyro[0];
        leg_struct->dis_x = leg->dynamic_get_deg()*_wheel_R/MOTOR_GEAR;
        leg_struct->dot_dis_x = leg->dynamic_get_speed()*_wheel_R/MOTOR_GEAR/60;
        float temp_leg = leg->_Phi0;
        float temp_theta = temp_leg - PI/2 - leg_struct->phi;
        float dot_theta = 0.4f*leg_struct->dot_theta + 0.6*(temp_theta - leg_struct->theta);
        leg_struct->theta = temp_theta;
        leg_struct->dot_theta = dot_theta;
    }
    void chassis_force_ctrl(float left_force, float left_tor, float left_dynamic, float right_force,float right_tor, float right_dynamic) {
        _left_leg->leg_force_ctrl(left_force,left_tor,left_dynamic);
        _right_leg->leg_force_ctrl(right_force,right_tor,right_dynamic);
    }
    leg_info left_leg, right_leg;
private:
    leg *_left_leg, *_right_leg;
    float32_t _left_data[6],_right_data[6];
    float32_t _left_out_put[2],_right_out_put[2];
    float32_t _data_k[12];
    float _wheel_R;
    const app_ins_data_t *_ins;
};
}

void app_chassis_init();
void app_chassis_task(void *argument);


#ifdef __cplusplus
}
#endif