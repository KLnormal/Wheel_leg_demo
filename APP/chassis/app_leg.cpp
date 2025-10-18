//
// Created by 15082 on 2025/10/18.
//

#include "app_leg.h"
#include "matrix.h"
void leg::leg::leg_ctrl(float32_t T, float32_t Tp, float32_t force_L0) {
    leg_status_clc();

    joint_A_->joint_ctrl(0);
    joint_E_->joint_ctrl(0);
    dynamic_->tor_ctrl(0);
}
void leg::leg::leg_init() {
    joint_A_->joint_init();
    joint_E_->joint_init();
    dynamic_->motor_init();
}
void leg::leg::leg_vmc_ctrl(float32_t T, float32_t Tp) {

}


void leg::leg::leg_status_clc() {
    //计算phi2,phi3
    my_leg_status_.xb = my_leg_status_.l1*cos(my_leg_status_.phi1), my_leg_status_.yb = my_leg_status_.l1*sin(my_leg_status_.phi1);
    my_leg_status_.xd = my_leg_status_.l_AE+my_leg_status_.l4 *cos(my_leg_status_.phi4), my_leg_status_.yd = my_leg_status_.l4*sin(my_leg_status_.phi4);
    const float A0 = 2*my_leg_status_.l2 *(my_leg_status_.xb-my_leg_status_.xd), B0 = 2*LEG_L2*(my_leg_status_.yb - my_leg_status_.yd),
        C0 = powf(my_leg_status_.xb-my_leg_status_.xd, 2) + pow(my_leg_status_.yb - my_leg_status_.yd, 2) - pow(LEG_L3,2)+pow(LEG_L2,2);
    const float32_t phi3_temp_y = (my_leg_status_.xb - my_leg_status_.xd) + my_leg_status_.l2*cos(my_leg_status_.phi2),
        phi3_temp_x = (my_leg_status_.yb - my_leg_status_.yd) + my_leg_status_.l2 * sin(my_leg_status_.phi2);
    my_leg_status_.phi3 = atan2(phi3_temp_y, phi3_temp_x);
    const float tempy =-2*B0 + sqrt(4*B0*B0-4*(C0*C0-A0*A0)), temp_x = 2*(C0-A0);
    my_leg_status_.phi2 = 2*atan2(tempy, temp_x);
    //计算正解算获取腿长与phi0
    my_leg_status_.phi1 = joint_A_->joint_deg_;
    my_leg_status_.phi4 = joint_E_->joint_deg_;
    my_leg_status_.leg_x = -my_leg_status_.l_AE/2+my_leg_status_.l1*cos(my_leg_status_.phi1)+my_leg_status_.l2*cos(my_leg_status_.phi2);
    my_leg_status_.leg_y = my_leg_status_.l1*sin(my_leg_status_.phi1) + my_leg_status_.l2*sin(my_leg_status_.phi2);
    my_leg_status_.L0 = sqrt(my_leg_status_.leg_x*my_leg_status_.leg_x + my_leg_status_.leg_y*my_leg_status_.leg_y);
    my_leg_status_.phi0 = atan2(my_leg_status_.leg_y, my_leg_status_.leg_x);
}
