//
// Created by 15082 on 2025/10/18.
//

#include "app_leg.h"
#include "matrix.h"
void leg::leg::leg_ctrl(float32_t tor_A, float32_t tor_B, float32_t tor_dynamic) {
    leg_status_clc();
}


void leg::leg::leg_status_clc() {
    //计算正解算获取腿长与phi0
    my_leg_status_.phi1 = joint_A_->joint_deg_;
    my_leg_status_.phi1 = joint_B_->joint_deg_;
    my_leg_status_.leg_x = -my_leg_status_.l_AE/2+my_leg_status_.l1*cos(my_leg_status_.phi1);
    my_leg_status_.leg_y = my_leg_status_.l1*sin(my_leg_status_.phi1) + my_leg_status_.l2*sin(my_leg_status_.phi2);
    my_leg_status_.L0 = sqrt(my_leg_status_.leg_x*my_leg_status_.leg_x + my_leg_status_.leg_y*my_leg_status_.leg_y);
    my_leg_status_.phi0 = atan2(my_leg_status_.leg_y, my_leg_status_.leg_x);
    //计算VMC相关数据
    float32_t data[4];
    data[0] = LEG_L1*sin(my_leg_status_.phi1-my_leg_status_.phi2)*sin(my_leg_status_.phi3)/sin(my_leg_status_.phi2 - my_leg_status_.phi3);
    data[1] = LEG_L4*sin(my_leg_status_.phi3 - my_leg_status_.phi4)*sin(my_leg_status_.phi2)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    data[2] = -LEG_L1*sin(my_leg_status_.phi1-my_leg_status_.phi2)*cos(my_leg_status_.phi3)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    data[3] = -LEG_L4*sin(my_leg_status_.phi3 - my_leg_status_.phi4)*cos(my_leg_status_.phi2)/sin(my_leg_status_.phi2-my_leg_status_.phi3);

}
