//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_sys.h"
#include "sys_task.h"
#include "app_wheel_leg_motor.h"
#include "bsp_uart.h"
#ifdef COMPILE_GIMBAL

// 静态任务，在 CubeMX 中配置

// Motor::DMMotor test("joint1",Motor::DMMotor::J4310,{
//         .slave_id = 0x21,
//         .master_id = 0x11,
//         .port = E_CAN2,
//         .mode = Motor::DMMotor::MIT,
//         .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
//     });
// wheel_leg_motor::joint joint1(&test,0.5,1);
Motor::DJIMotor test("dynamic",Motor::DJIMotor::M3508,{.id = 0x01, .port = E_CAN1, .mode = Motor::DJIMotor::CURRENT});
wheel_leg_motor::dynamic dynamic_1(&test,1);
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);
    dynamic_1.motor_init();
    OS::Task::SleepMilliseconds(1000);
    while(true) {
        dynamic_1.motor_deg_clc();
        bsp_uart_printf(E_UART_DEBUG,"%f\n",dynamic_1.total_deg);
        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {

}

#endif
