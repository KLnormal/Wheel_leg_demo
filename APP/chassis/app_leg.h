//
// Created by 15082 on 2025/10/18.
//

#ifndef APP_LEG_H
#define APP_LEG_H
#include "app_wheel_leg_datasheet.h"
#include "app_wheel_leg_motor.h"
#endif //APP_LEG_H

namespace leg {
struct leg_status {
    float32_t l_AE, l1, l2;
    float32_t phi0,phi1,phi2,phi3,phi4;
    float32_t L0;
    float32_t leg_x, leg_y;
};
struct status_space {
    float32_t theta, dot_theta;
    float32_t distance_x, dot_distance_x;
    float32_t phi, phi_dot;
};
class leg {
    public:
    leg();
    leg(wheel_leg_motor::joint *joint_A, wheel_leg_motor::joint *joint_B, wheel_leg_motor::dynamic *dynamic)
        : joint_A_(joint_A), joint_B_(joint_B), dynamic_(dynamic) {
        my_leg_status_.l_AE = LEG_AE;
        my_leg_status_.l1 = LEG_L1;
        my_leg_status_.l2 = LEG_L2;
    }
    void leg_ctrl(float32_t tor_A, float32_t tor_B, float32_t tor_dynamic);
    void leg_status_clc();
    void leg_vmc_ctrl(float32_t t, float32_t tp);
    private:
    wheel_leg_motor::joint *joint_A_, *joint_B_;
    wheel_leg_motor::dynamic *dynamic_;
    status_space my_status_space_;
    leg_status my_leg_status_;
};
}