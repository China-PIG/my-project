#include "stm32f1xx_hal.h" 
#include "tim.h"
extern uint8_t currentAngle;  
void Servo_SetAngle(uint8_t angle)
{
   if (angle > 180) angle = 180;
   uint16_t pulse = 500 + ((2000 * angle) / 180);
   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
	currentAngle = angle; 
}





