//
// Created by 15082 on 2025/10/19.
//

#ifndef APP_WHELL_LEG_H
#define APP_WHELL_LEG_H
#include "app_leg.h"


//我们定义，腿的坐标系和王工的一样，方向为总左边看过去
/*             喵板
 *   joint4 |------| joint1
 *           |    |                 <----看过去为王工坐标系视角
 *           |    |
 *   joint3 |------| joint2
 *            分电板
 *            |
 *            |
 *            | x正方向，前进方向
 *            v
 */

namespace total {
typedef enum {
    Left_leg,
    Right_leg
}leg_direction;
struct chassis {
    float32_t left_T, left_Tp; //T为驱动轮扭矩，Tp为髋关节扭矩
    float32_t right_T, right_Tp;
    float32_t target_distance_left, target_distance_right;
};
struct simple_PID {
    float32_t target, current, old;
    float32_t Kp, Ki, Kd;
    float32_t temp_p, temp_i, temp_d;
    float32_t error,old_error, sum;
    float32_t I_limit, Sum_limit;
};
class wheel_leg {
    public:
    wheel_leg();
    wheel_leg(leg::leg* left_leg, leg::leg* right_leg, const app_ins_data_t* ins): left_leg_(left_leg), right_leg_(right_leg), ins_(ins) {
    };
private:
    float leg_length_ctrl(float length, leg_direction leg_switch);
    float leg_combine_ctrl();
    float PID(float target, float current, float Kp, float Ki, float Kd, float I_limit, float sum_limit, leg_direction leg_switch);
    leg::leg* left_leg_;
    leg::leg* right_leg_;
    const app_ins_data_t* ins_;
    simple_PID left_length_PD, right_length_PD;

};
}
#endif //APP_WHELL_LEG_H
