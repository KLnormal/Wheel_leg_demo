//
// Created by 15082 on 2025/10/2.
//

#include "app_leg_forward.h"

#include "app_leg_motor.h"
#include "app_wheel_leg_datalist.h"

#include <cmath>

void forward_clc(float theta1, float theta4) {
    float xb = LEG_L1*cos(theta1), yb = LEG_L1*sin(theta1);
    float xd = LEG_L5+LEG_L4*cos(theta4), yd = LEG_L4*sin(theta4);
    float A0 = 2*LEG_L2*(xb-xd), B0 = 2*LEG_L2*(yb - yd), C0 = pow(xb-xd, 2) + pow(yb - yd, 2) - pow(LEG_L3,2)+pow(LEG_L2,2);
    float tempy =-2*B0 - sqrt(4*B0*B0-4*(C0*C0-A0*A0)), temp_x = 2*(C0-A0);
    float t = tempy/temp_x;
    float theta2 = 2*atan2(tempy, temp_x);
    float x0 = -LEG_L5/2+LEG_L1*cos(theta1)+LEG_L2*cos(theta2);
    float y0 = LEG_L1*sin(theta1)+LEG_L2*sin(theta2);
    float L0 = sqrt(pow(x0,2) + pow(y0,2));
    float theta0 = atan2(y0, x0);
}