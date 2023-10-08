#ifndef __MOTOR
#define __MOTOR



#include "stm32f1xx_hal.h"
#include "mytype.h"
#include "can.h"



#define FILTER_BUF_LEN		5
/*电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;

#define motor_num 1		//电机数量

extern moto_measure_t  moto_chassis[];
extern moto_measure_t moto_info;

u8 get_moto_measure(moto_measure_t *ptr, uint8_t* can_receive_data);
u8 set_moto_current(CAN_HandleTypeDef hcan, s16 SID, s16 iq1);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
void get_total_angle(moto_measure_t *p);
float angle_speed_cacl(float set_round);

#endif
