#ifndef SHOOT_H
#define SHOOT_H
#include "sys.h"
#include "CAN_receive.h"

extern u8 Fire_mode;
extern int block_flag;
extern int16_t trigger_moto_speed_ref2;

void Encode_C(M_Data*ptr);
void shoot_task(void);
void shoot_task1(void);
void block_bullet_handle(void);
void shoot_ready_control(void);

#endif

