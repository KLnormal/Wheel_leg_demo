//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_ins.h"
#include "app_sys.h"
#include "bsp_def.h"
#include "bsp_time.h"
#include "sys_task.h"
#include "SJTU_Matrix/matrix.h"
#include "bsp_uart.h"

#ifdef COMPILE_CHASSIS

void Chassis_Wheel_Leg::leg::leg_force_ctrl(float force, float tor, float dynamic_tor) {
    leg_ctrl(vmc_tor1,vmc_tor2);
    joint_get_polar();
    float temp_y, temp_x;
    temp_y = (_xb - _xd) + LEG_L2*cos(_phi2);
    temp_x = (_yb - _yd) + LEG_L2*sin(_phi2);
    _phi3 = atan2(temp_y, temp_x);
    float32_t data[4];
    data[0] = LEG_L1*sin(_phi1-_phi2)*sin(_phi3)/sin(_phi2 - _phi3);
    data[1] = LEG_L4*sin(_phi3 - _phi4)*sin(_phi2)/sin(_phi2-_phi3);
    data[2] = -LEG_L1*sin(_phi1-_phi2)*cos(_phi3)/sin(_phi2-_phi3);
    data[3] = -LEG_L4*sin(_phi3 - _phi4)*cos(_phi2)/sin(_phi2-_phi3);
    Matrixf<2,2> Jacobi(data);
    Matrixf<2,2> Jacobi_T = Jacobi.trans();
    float32_t vector[2] = {force,tor};
    Matrixf<2,1> target(vector);
    data[0] = 0;
    data[1] = -1/_L0;
    data[2] = 1;
    data[3] = 0;
    Matrixf<2,2> M(data);
    data[0] = cos(_Phi0-PI/2);
    data[1] = -sin(_Phi0-PI/2);
    data[2] = sin(_Phi0-PI/2);
    data[3] = cos(_Phi0-PI/2);
    Matrixf<2,2> R(data);
    Matrixf<2,1> answer;
    answer = Jacobi_T*R*M*target;
    vmc_tor1 = answer[0][0];
    vmc_tor2 = answer[1][0];
    _motor->motor_tor(dynamic_tor);
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
    _L0 = sqrt(pow(x0,2) + pow(y0,2));
    _Phi0 = atan2(y0, x0);
    _x0 = x0;
    _y0 = y0;
    _phi2 = phi2;
    _xb = xb, _yb = yb, _xd = xd, _yd = yd;
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
    },1,-std::numbers::pi/2,7);
joint right_joint2("joint2",Motor::DMMotor::J4310,{
        .slave_id = 0x22,
        .master_id = 0x12,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },1,-std::numbers::pi/2,7);
joint left_joint3("joint3",Motor::DMMotor::J4310,{
        .slave_id = 0x23,
        .master_id = 0x13,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },-1,std::numbers::pi/2,7);
joint left_joint4("joint4",Motor::DMMotor::J4310,{
        .slave_id = 0x24,
        .master_id = 0x14,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    },-1,std::numbers::pi/2,7);

float K[12] = {-3.897652, 6.082493, -0.264892, 0.581049, -0.784465, 2.864459, -0.762822, 2.583690, 2.640943, 5.637764, 0.357547, 0.368631};

float32_t fit_K[12];
dynamic_motor right_motor("right_motor",Motor::DJIMotor::M3508,{1,E_CAN1,Motor::DJIMotor::CURRENT},-1);
dynamic_motor left_motor("left_motor",Motor::DJIMotor::M3508,{2,E_CAN1,Motor::DJIMotor::CURRENT},1);
Chassis_Wheel_Leg::leg right_leg(&right_joint1,&right_joint2,&right_motor);
Chassis_Wheel_Leg::leg left_leg(&left_joint4,&left_joint3,&left_motor);
void k_fit_function(float32_t len) {
    auto l1 = len, l2 = l1 * len, l3 = l2 * len, l4 = l3 * len;
    fit_K[0] = (-2361.07f)*l4 + (1225.31f)*l3 + (-166.903f)*l2 + (-21.2382f)*l1 + (-0.539707f);
    fit_K[1] = (-201.808f)*l4 + (133.379f)*l3 + (-32.2669f)*l2 + (0.607461f)*l1 + (-0.055765f);
    fit_K[2] = (-1745.48f)*l4 + (875.154f)*l3 + (-135.319f)*l2 + (1.52154f)*l1 + (-0.174626f);
    fit_K[3] = (-1543.18f)*l4 + (785.215f)*l3 + (-126.416f)*l2 + (2.3188f)*l1 + (-0.228859f);
    fit_K[4] = (-1119.4f)*l4 + (830.556f)*l3 + (-204.58f)*l2 + (10.371f)*l1 + (3.12765f);
    fit_K[5] = (166.589f)*l4 + (-68.5338f)*l3 + (9.60774f)*l2 + (-1.61931f)*l1 + (0.495759f);
    fit_K[6] = (-3206.11f)*l4 + (2798.12f)*l3 + (-892.033f)*l2 + (112.722f)*l1 + (1.09372f);
    fit_K[7] = (366.481f)*l4 + (-151.727f)*l3 + (9.88513f)*l2 + (2.8917f)*l1 + (0.254715f);
    fit_K[8] = (-1566.49f)*l4 + (1099.86f)*l3 + (-262.553f)*l2 + (14.4538f)*l1 + (3.32308f);
    fit_K[9] = (-259.57f)*l4 + (334.821f)*l3 + (-98.6408f)*l2 + (1.23825f)*l1 + (3.19796f);
    fit_K[10] = (12123.3f)*l4 + (-6033.4f)*l3 + (905.209f)*l2 + (-0.651238f)*l1 + (0.640345f);
    fit_K[11] = (1056.55f)*l4 + (-526.682f)*l3 + (75.1614f)*l2 + (2.15048f)*l1 + (-0.273727f);
}

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    // right_leg.leg_init();
    // left_leg.leg_init();
    // float data;
    // left_motor.motor_init();
    // right_motor.motor_init();
    const auto ins = app_ins_data();
    OS::Task::SleepMilliseconds(3000);
    Chassis_Wheel_Leg::chassis my_chassis(&left_leg,&right_leg,K,ins,WHEEL_R);
    my_chassis.chassis_init();
    float32_t tor_ml, tor_mr, tor_l, tor_r;
    uint64_t st, ed;
    float32_t state_left[6], state_right[6];
    float32_t out_left[2], out_right[2];
	while(true) {
	    // left_motor.motor_tor(0);
	    // float deg = left_leg.dynamic_get_deg();
	    // float deg_r = right_leg.dynamic_get_deg();
	    // right_leg.leg_force_ctrl(-12,0,0);
	    // left_leg.leg_force_ctrl(-12,0,0);
	    // bsp_uart_printf(E_UART_DEBUG,"%f,%f\n",deg,deg_r);

        st = bsp_time_get_us();

	    if(abs(my_chassis.left_leg_struct.phi) > std::numbers::pi/4) {
	        my_chassis.chassis_force_ctrl(0,0,0,0,0,0);
	        OS::Task::SleepMilliseconds(10);
	        BSP_ASSERT(false);
	    }
	    tor_r = my_chassis._right_out_put[1];
	    tor_mr = my_chassis._right_out_put[0];
	    tor_l = my_chassis._left_out_put[1];
	    tor_ml = my_chassis._left_out_put[0];
	    Chassis_Wheel_Leg::chassis::leg_length(0.12,my_chassis.right_force,my_chassis._right_leg,my_chassis.old_left_len,my_chassis.right_len);
	    Chassis_Wheel_Leg::chassis::leg_length(0.12,my_chassis.left_force,my_chassis._left_leg,my_chassis.old_left_len,my_chassis.left_len);
	    my_chassis.chassis_force_ctrl(my_chassis.left_force,my_chassis._left_out_put[1],my_chassis._left_out_put[0],my_chassis.right_force,my_chassis._right_out_put[1],my_chassis._right_out_put[0]);
	    // my_chassis.chassis_force_ctrl(0,0,0,0,0,0);
	    my_chassis.chassis_lqr_clc();
	    my_chassis.leg_combine();
        memcpy(state_left,my_chassis._left_data,sizeof(float32_t)*6);
	    memcpy(state_right,my_chassis._right_data,sizeof(float32_t)*6);
	    memcpy(out_left,my_chassis._left_out_put,sizeof(float32_t)*2);
	    bsp_uart_printf(E_UART_DEBUG,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",state_left[0],state_left[1],state_left[2],state_left[3],state_left[4],state_left[5],state_right[0],state_right[1],state_right[2],state_right[3],state_right[4],state_right[5]);
	    // bsp_uart_printf(E_UART_DEBUG,"%f,%f\n",out_left[0],out_left[1]);
	    // ed = bsp_time_get_us();
	    // bsp_uart_printf(E_UART_DEBUG, "%lld\r\n", ed - st);

		OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {

}

#endif