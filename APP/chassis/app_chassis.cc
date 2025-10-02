//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include "app_sys.h"
#include "sys_task.h"
#include "app_wheel_leg_datalist.h"
#include "app_leg_forward.h"
#include "app_leg_motor.h"
#include "bsp_uart.h"

#ifdef COMPILE_CHASSIS

// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
    leg_init();
    float* temp_deg = leg_deg();
	while(true) {
	    leg_deg();
	    bsp_uart_printf(E_UART_DEBUG, "%f,%f\n",temp_deg[0],temp_deg[1]);
		OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {

}

#endif