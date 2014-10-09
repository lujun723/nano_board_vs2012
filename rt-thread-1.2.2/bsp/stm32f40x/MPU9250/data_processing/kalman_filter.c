/*
 * File      : kalman_filter.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2014-09-18     Jun.Lu      	the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/
#include "stm32f4xx.h"

//Q越大，R越小：则滤波输出动态性更好，收敛性变差；
//Q越小，R越大：则滤波输出动态性变差，收敛性变好；
//如果采样频率慢：要增大Q，减小R，才有更好的实时性；
//如果采样频率快：可以减小Q，增大R，使收敛性更好。
//
float P=10.0,R=20.0,Q=0.001;

int16_t kalman(int16_t sensor_val,uint8_t *state,int16_t kalmanPoint[])
{
	int16_t ret=0;
	double x_mid,p_mid,kg;
	switch(*state)
	{
		case 0:
			kalmanPoint[0] = sensor_val;
			ret=kalmanPoint[0];
			(*state)=1;
			break;
		case 1:
			x_mid = kalmanPoint[0];
			p_mid = (P + Q);
			kg = (p_mid / (p_mid + R));
			kalmanPoint[1]= x_mid + kg * (sensor_val - x_mid);
			P = ((1 - kg) * p_mid);
			ret=kalmanPoint[1];
			kalmanPoint[0]=kalmanPoint[1];
			break;
		default:
			break;	
	}
	
	return (ret);
}


/*@}*/
