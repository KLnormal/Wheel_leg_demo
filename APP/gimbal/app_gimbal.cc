//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_leg.h"
#include "app_sys.h"
#include "sys_task.h"
#include "app_wheel_leg_motor.h"
#include "bsp_uart.h"
#ifdef COMPILE_GIMBAL

// 静态任务，在 CubeMX 中配置

Motor::DMMotor motor1("joint1",Motor::DMMotor::J4310,{
        .slave_id = 0x21,
        .master_id = 0x11,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    });
Motor::DMMotor motor2("joint2",Motor::DMMotor::J4310,{
        .slave_id = 0x22,
        .master_id = 0x12,
        .port = E_CAN2,
        .mode = Motor::DMMotor::MIT,
        .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
    });
wheel_leg_motor::joint left_A(&motor1,-PI_F32/2,1);
wheel_leg_motor::joint left_E(&motor2,-PI_F32/2,1);
Motor::DJIMotor test("dynamic",Motor::DJIMotor::M3508,{.id = 0x01, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT});
wheel_leg_motor::dynamic dynamic_1(&test,1);
leg::leg left_leg(&left_A,&left_E,&dynamic_1);

void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    left_leg.leg_init();
    OS::Task::SleepMilliseconds(1000);
    while(true) {
        left_leg.leg_ctrl(0,0,0);
        bsp_uart_printf(E_UART_DEBUG,"%f,%f\n",left_leg.my_leg_status_.L0,left_leg.my_leg_status_.phi0);
        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {

}

#endif
