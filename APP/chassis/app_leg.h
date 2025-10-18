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
    float32_t l_AE, l1, l2,l3,l4;
    float32_t phi0,phi1,phi2,phi3,phi4;
    float32_t L0;
    float32_t leg_x, leg_y;
    float32_t xd, yd, xb, yb;
};
struct status_space {
    float32_t theta, dot_theta;
    float32_t distance_x, dot_distance_x;
    float32_t phi, phi_dot;
};
class leg {
    public:
    leg();
    leg(wheel_leg_motor::joint *joint_A, wheel_leg_motor::joint *joint_E, wheel_leg_motor::dynamic *dynamic)
        : joint_A_(joint_A), joint_E_(joint_E), dynamic_(dynamic) {
        my_leg_status_.l_AE = LEG_AE;
        my_leg_status_.l1 = my_leg_status_.l4 = LEG_L1;
        my_leg_status_.l2 = my_leg_status_.l3 = LEG_L2;
    }
    void leg_init();
    void leg_ctrl(float32_t T, float32_t Tp, float32_t force_L0);
    void leg_status_clc();
    void leg_vmc_ctrl(float32_t T, float32_t Tp);
    status_space my_status_space_;
    leg_status my_leg_status_;
    private:
    wheel_leg_motor::joint *joint_A_, *joint_E_;
    wheel_leg_motor::dynamic *dynamic_;

};
}