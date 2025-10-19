//
// Created by 15082 on 2025/10/18.
//

#include "app_leg.h"

#include "app_ins.h"
#include "matrix.h"
void leg::leg::leg_ctrl(float32_t T, float32_t Tp, float32_t force_L0) {
    leg_status_clc();
    leg_vmc_ctrl(Tp,force_L0);
    joint_A_->joint_ctrl(my_status_space_.motor_joint_tor_A);
    joint_E_->joint_ctrl(my_status_space_.motor_joint_tor_E);
    dynamic_->tor_ctrl(T);
    leg::leg::status_space_clc();
}
void leg::leg::leg_init() const {
    joint_A_->joint_init();
    joint_E_->joint_init();
    dynamic_->motor_init();
}
void leg::leg::leg_vmc_ctrl( float32_t Tp, float32_t force_L0) {
    //这里的Tp传入的是髋关节的扭矩，但是我们VMC生成的是末端的扭矩，传递过去方向相反，所以说要在后面加-Tp
    float32_t data[4];
    data[0] = my_leg_status_.l1*sin(my_leg_status_.phi1-my_leg_status_.phi2)*sin(my_leg_status_.phi3)/sin(my_leg_status_.phi2 - my_leg_status_.phi3);
    data[1] = my_leg_status_.l4*sin(my_leg_status_.phi3 - my_leg_status_.phi4)*sin(my_leg_status_.phi2)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    data[2] = -my_leg_status_.l1*sin(my_leg_status_.phi1-my_leg_status_.phi2)*cos(my_leg_status_.phi3)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    data[3] = -my_leg_status_.l4*sin(my_leg_status_.phi3 - my_leg_status_.phi4)*cos(my_leg_status_.phi2)/sin(my_leg_status_.phi2-my_leg_status_.phi3);
    Matrixf<2,2> Jacobi(data);
    Matrixf<2,2> Jacobi_T = Jacobi.trans();
    float32_t vector[2] = {force_L0,-Tp};
    Matrixf<2,1> target(vector);
    data[0] = 0;
    data[1] = -1/my_leg_status_.L0;
    data[2] = 1;
    data[3] = 0;
    Matrixf<2,2> M(data);
    data[0] = cos(my_leg_status_.phi0-PI/2);
    data[1] = -sin(my_leg_status_.phi0-PI/2);
    data[2] = sin(my_leg_status_.phi0-PI/2);
    data[3] = cos(my_leg_status_.phi0-PI/2);
    Matrixf<2,2> R(data);
    Matrixf<2,1> answer;
    answer = Jacobi_T*R*M*target;
    my_status_space_.motor_joint_tor_A = answer[0][0];
    my_status_space_.motor_joint_tor_E = answer[1][0];
}
void leg::leg::status_space_clc(){
    my_status_space_.distance_x = this->dynamic_->get_deg()/MOTOR_GEAR*WHEEL_R;
    my_status_space_.dot_distance_x = this->dynamic_->get_v()/MOTOR_GEAR*WHEEL_R;
    my_status_space_.phi = -ins_->roll/180*PI_F32;
    my_status_space_.phi_dot = -ins_->raw.gyro[0];
    my_status_space_.old_theta = my_status_space_.theta;
    my_status_space_.theta = -(PI_F32/2-(my_leg_status_.phi0+my_status_space_.phi));
    my_status_space_.dot_theta =Mid_avg_filter(0.8f*my_status_space_.dot_theta + 0.2f*(my_status_space_.theta - my_status_space_.old_theta)*1000);
}
//源文件
/*******************************************************************************
 * @fn Mid_avg_filter
 * @brief 实现中值滤波
 * @param filter 滤波器
 * @param data 最新数据
 * @return 滤波后的结果
 ******************************************************************************/
float32_t leg::leg::Mid_avg_filter(const float32_t data)
{
    float32_t tem;
    uint16_t i, j;
    static mid_filter f_tmp;  //临时副本，用于排序
    //采用循环队列形式将最新数据入队
    my_mid_filter_.dataBuf[my_mid_filter_.index] = data;
    my_mid_filter_.index = (my_mid_filter_.index + 1) % MID_AVG_SIZE;

    //采用冒泡法将滤波器内数据升序排序
    f_tmp=my_mid_filter_;
    for (i = 0; i < MID_AVG_SIZE - 1; i ++) //MID_AVG_FILTER_SIZE-1不用与自己比较
    {
        uint16_t count = 0;
        for (j = 0; j < MID_AVG_SIZE - 1 - i; j++)
        {
            if (f_tmp.dataBuf[j] > f_tmp.dataBuf[j + 1])
            {
                tem = f_tmp.dataBuf[j];
                f_tmp.dataBuf[j] = f_tmp.dataBuf[j + 1];
                f_tmp.dataBuf[j + 1] = tem;
                count = 1;
            }
        }
        if (count == 0)			//如果某一趟没有交换位置，则说明已经排好序，直接退出循环
            break;
    }
    //取中值
    if(MID_AVG_SIZE%2==0)//判断奇偶
    {
        return (f_tmp.dataBuf[MID_AVG_SIZE/2]+f_tmp.dataBuf[(MID_AVG_SIZE/2)-1])/2;
    }
    return f_tmp.dataBuf[(MID_AVG_SIZE-1)/2];
}
void leg::leg::leg_status_clc() {
    //计算phi2,phi3
    my_leg_status_.xb = my_leg_status_.l1*cos(my_leg_status_.phi1), my_leg_status_.yb = my_leg_status_.l1*sin(my_leg_status_.phi1);
    my_leg_status_.xd = my_leg_status_.l_AE+my_leg_status_.l4 *cos(my_leg_status_.phi4), my_leg_status_.yd = my_leg_status_.l4*sin(my_leg_status_.phi4);
    const float A0 = 2*my_leg_status_.l2 *(my_leg_status_.xb-my_leg_status_.xd), B0 = 2*LEG_L2*(my_leg_status_.yb - my_leg_status_.yd),
        C0 = powf(my_leg_status_.xb-my_leg_status_.xd, 2) + powf(my_leg_status_.yb - my_leg_status_.yd, 2) - powf(LEG_L3,2)+powf(LEG_L2,2);
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
