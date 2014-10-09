/**
	********************************************************************
	*	@file			mpu9250_api.h
	*	@author			Jun.Lu
	*	@version 		V1.0.0
	*	@date			14-Jun-2014
	*	@brief			
	********************************************************************
	*	@attention
	*	
	*-------------------------------------------------------------------
	* Pins definition:
	*	GPIO:
	*	PIN					FUNCTION				NOTES
	*
	*	Change Logs:
	*	Date					Author					Notes
	*	
	*
	********************************************************************
	*/

/* Define to prevent recursive inclusion ---------------------------*/
#ifndef __MPU9250_API_H__
#define __MPU9250_API_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
/*	Includes -------------------------------------------------------*/
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
  float TrueX;
  float TrueY;
  float TrueZ;
  int16_t Kalman_X;
  int16_t Kalman_Y;
  int16_t Kalman_Z;
} SENSOR_9AXIS;

enum SENSOR_CHANNEL
{
	ACC_X=0,
	ACC_Y,
	ACC_Z,
	GYR_X,
	GYR_Y,
	GYR_Z
};
/*	Private macro --------------------------------------------------*/
#define Byte16(ByteH, ByteL) ((u16)(((ByteH)<<8) | (ByteL)))

/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /*__MPU9250_API_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
