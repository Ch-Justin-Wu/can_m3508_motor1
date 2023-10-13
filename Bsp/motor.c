#include "can.h"
#include "motor.h"
#include "pid.h"

moto_measure_t moto_chassis[2] = {0}; // 2 chassis moto
moto_measure_t moto_info;

/**
 * @brief Get the moto measure object 接收m3508电机通过CAN发过来的信息
 *
 * @param ptr
 * @return u8
 */
u8 get_moto_measure(moto_measure_t *ptr, uint8_t *can_receive_data)
{
    ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(can_receive_data[0] << 8 | can_receive_data[1]);
    ptr->real_current = (int16_t)(can_receive_data[2] << 8 | can_receive_data[3]);
    ptr->speed_rpm = ptr->real_current;
    ptr->given_current = (int16_t)(can_receive_data[4] << 8 | can_receive_data[5]) / -5;
    ptr->hall = can_receive_data[6];
    if (ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->angle - ptr->last_angle < -4096)
        ptr->round_cnt++;
    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle; //累计转过角度计算
    return can_rx_message.DLC;
}

/**
 * @brief Set the moto current object
 *        CAN发送电机电流数据帧
 *
 * @param hcan
 * @param SID
 * @param iq1
 * @return u8
 */
u8 set_moto_current(CAN_HandleTypeDef hcan, s16 SID, s16 iq1)
{

    uint32_t send_mail_box;
    can_tx_message.StdId = 0x200; // ID
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x04;

    can_send_data[0] = iq1 >> 8;
    can_send_data[1] = iq1;

    if (HAL_CAN_AddTxMessage(&hcan, &can_tx_message, can_send_data, &send_mail_box) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief Get the moto offset object
 * 获取电机转子起始角度
 *
 * @param ptr
 * @param hcan
 */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef *hcan)
{
    ptr->angle = (uint16_t)(can_receive_data[0] << 8 | can_receive_data[1]);
    ptr->offset_angle = ptr->angle;
}

#define ABS(x) ((x > 0) ? (x) : (-x))

/**
 * @brief Get the total angle object
 * 电机上电角度=0， 之后用这个函数更新3508电机的相对开机后（为0）的相对角度。
 *
 * @param p
 */
// void get_total_angle(moto_measure_t *p)
// {

//     int res1, res2, delta;
//     if (p->angle < p->last_angle)
//     {                                           //可能的情况
//         res1 = p->angle + 8192 - p->last_angle; //正转，delta=+
//         res2 = p->angle - p->last_angle;        //反转	delta=-
//     }
//     else
//     {                                           // angle > last
//         res1 = p->angle - 8192 - p->last_angle; //反转	delta -
//         res2 = p->angle - p->last_angle;        //正转	delta +
//     }
//     //不管正反转，肯定是转的角度小的那个是真的
//     if (ABS(res1) < ABS(res2))
//         delta = res1;
//     else
//         delta = res2;

//     p->total_angle += delta;
//     p->last_angle = p->angle;
// }

//此处只有一个电机,若多个电机不可这样使用
u8 i=0;

//设定圈数
// float set_round=10;
float real_total_angle;
float angle_setspeed;
float set_angle;
//float actual_round;
/**
 * @brief 角速度计算
 *
 * @param set_round
 * @return float
 */
float angle_speed_cacl(float set_round)
{
    set_angle = set_round * 360.0f * 3591.0f / 187.0f;                  //减速比3591/187  计算设定总角度 36.0f/1.0f
    real_total_angle = moto_chassis[i].total_angle / 8192.0f * 360.0f;  //换算实际总角度
    //actual_round=(float)moto_chassis[i].total_angle / 8192.0f * 187.0f  / 3591.0f;
    angle_setspeed = pid_calc(&pid_angle, real_total_angle, set_angle); //通过角度环计算设定(角)速度
    return angle_setspeed;
}

