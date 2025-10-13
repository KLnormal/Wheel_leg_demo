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
#define LEG_FORWARD 12.0f
#define LEG_P 100
#define LEG_D 15
#define COMBINE_P 5
#define COMBINE_D 1
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
    void leg_disable() {
        _joint1->joint_disable();
        _joint2->joint_disable();
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
        memcpy(_data_k,K,sizeof(float32_t)*12);
    };
    void chassis_init() {
        _left_leg->leg_init();
        _right_leg->leg_init();
    }
    void chassis_disable() {
        _left_leg->leg_disable();
        _right_leg->leg_disable();
    }
    void chassis_clc(leg *leg, leg_info *leg_struct,float32_t *data) {
        leg_struct->phi = -_ins->roll/180.0f*M_PI;
        leg_struct->dot_phi = -_ins->raw.gyro[0]/180.0f*M_PI;
        leg_struct->dis_x = leg->dynamic_get_deg()*_wheel_R/MOTOR_GEAR;
        leg_struct->dot_dis_x = leg->dynamic_get_speed()*_wheel_R/MOTOR_GEAR/60;
        float temp_leg = leg->_Phi0;
        float temp_theta = -(PI/2-(temp_leg+leg_struct->phi));
        float dot_theta = 0.4f*leg_struct->dot_theta + 0.6*(temp_theta - leg_struct->theta);
        leg_struct->theta = temp_theta;
        leg_struct->dot_theta = dot_theta;
        data[0] = leg_struct->theta;
        data[1] = leg_struct->dot_theta;
        data[2] = leg_struct->dis_x;
        data[3] = leg_struct->dot_dis_x;
        data[4] = leg_struct->phi;
        data[5] = leg_struct->dot_phi;
    }
    void chassis_force_ctrl(float left_force, float left_tor, float left_dynamic, float right_force,float right_tor, float right_dynamic) {
        _left_leg->leg_force_ctrl(left_force,left_tor,left_dynamic);
        _right_leg->leg_force_ctrl(right_force,right_tor,right_dynamic);
        chassis_clc(_left_leg,&left_leg_struct,_left_data);
        chassis_clc(_right_leg,&right_leg_struct,_right_data);
    }
    void chassis_lqr_clc() {
        _left_data[2] -= left_target_dis;
        _right_data[2] -= right_target_dis;
        left_target_dis = left_leg_struct.dis_x;
        right_target_dis = right_leg_struct.dis_x;
        Matrixf<2,6> K(_data_k);
        Matrixf<6,1> left_state(_left_data);
        Matrixf<2,1> left_temp = K*left_state;
        Matrixf<6,1> right_state(_right_data);
        Matrixf<2,1> right_temp = K*right_state;
        _left_out_put[0] = left_temp[0][0];
        _left_out_put[1] = left_temp[1][0];
        _right_out_put[0] = right_temp[0][0];
        _right_out_put[1] = right_temp[1][0];

    }
    static void leg_length(float32_t target, float32_t &force, leg *my_leg, float32_t &old, float32_t &current) {
        old = current;
        current = my_leg->_L0;
        float32_t output = LEG_FORWARD+(target-current)*LEG_P-(current-old)*LEG_D;
        force = output;
    }
    void leg_combine() {
        // float32_t temp_delta = _right_leg->_Phi0 - _left_leg->_Phi0;
        // float32_t temp_fix;
        // temp_fix = temp_delta*COMBINE_P;
        // _left_out_put[1] += (temp_fix + (temp_delta - old_delta_phi)*COMBINE_D);
        // _right_out_put[1] -= (temp_fix + (temp_delta - old_delta_phi)*COMBINE_D);
    }
    leg_info left_leg_struct, right_leg_struct;
    float32_t _left_out_put[2],_right_out_put[2];
    float32_t left_force = 0, right_force = 0;
    leg *_left_leg, *_right_leg;
    float32_t left_len, old_left_len, right_len, old_right_len;
    float32_t delta_phi, old_delta_phi;
    float32_t left_target_dis = 0, right_target_dis = 0;
private:
    float32_t _left_data[6],_right_data[6];
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