//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_sys.h"
#include "sys_task.h"

#include "bsp_uart.h"

#ifdef COMPILE_CHASSIS

void Chassis_Wheel_Leg::leg::leg_force_ctrl(float force_x, float force_y) {

}
void Chassis_Wheel_Leg::leg::joint_get_polar() {
    float xb = LEG_L1*cos(_phi1), yb = LEG_L1*sin(_phi1);
    float xd = LEG_L5+LEG_L4*cos(_phi4), yd = LEG_L4*sin(_phi4);
    float A0 = 2*LEG_L2*(xb-xd), B0 = 2*LEG_L2*(yb - yd), C0 = pow(xb-xd, 2) + pow(yb - yd, 2) - pow(LEG_L3,2)+pow(LEG_L2,2);
    float tempy =-2*B0 + sqrt(4*B0*B0-4*(C0*C0-A0*A0)), temp_x = 2*(C0-A0);
    float t = tempy/temp_x;
    float phi2 = 2*atan2(tempy, temp_x);
    float x0 = -LEG_L5/2+LEG_L1*cos(_phi1)+LEG_L2*cos(phi2);
    float y0 = LEG_L1*sin(_phi1)+LEG_L2*sin(phi2);
    _x0 = x0;
    _y0 = y0;
    _L0 = sqrt(pow(x0,2) + pow(y0,2));
    _Phi0 = atan2(y0, x0);
}
void  Chassis_Wheel_Leg::leg::leg_ctrl(float tor1, float tor2) {
    _joint1->joint_ctrl(tor1);
    _phi1 = _joint1->deg_clc();
    _joint2->joint_ctrl(tor2);
    _phi4 = _joint2->deg_clc();
}

joint right_joint1("joint1",Motor::DMMotor::J4310,{
        .slave_id = 0x21,
        .master_id = 0x11,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },1,-std::numbers::pi/2,2);
joint right_joint2("joint2",Motor::DMMotor::J4310,{
        .slave_id = 0x22,
        .master_id = 0x12,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },1,-std::numbers::pi/2,2);
joint left_joint3("joint3",Motor::DMMotor::J4310,{
        .slave_id = 0x23,
        .master_id = 0x13,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },-1,std::numbers::pi/2,2);
joint left_joint4("joint4",Motor::DMMotor::J4310,{
        .slave_id = 0x24,
        .master_id = 0x14,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },-1,std::numbers::pi/2,2);

dynamic_motor right_motor('a');
dynamic_motor left_motor('b');
Chassis_Wheel_Leg::leg right_leg(&right_joint1,&right_joint2,&right_motor);
Chassis_Wheel_Leg::leg left_leg(&left_joint4,&left_joint3,&left_motor);


// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    right_leg.leg_init();
    left_leg.leg_init();
	while(true) {
	    left_leg.leg_ctrl(0,0);
	    right_leg.leg_ctrl(0,0);
	    right_leg.joint_get_polar();
	    left_leg.joint_get_polar();
	    bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f\n",right_leg._Phi0,right_leg._y0,left_leg._Phi0,left_leg._y0);
		OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {

}

#endif