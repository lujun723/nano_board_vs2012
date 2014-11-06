/**
	********************************************************************
	*	@file		mpu9250_app.c
	*	@author		Jun.Lu
	*	@version	V1.0.0
	*	@date		14-Jun-2014
	*	@brief		
	********************************************************************
	*	@attention
	*	Using SPI for communicate to mpu9250.
	*-----------------------------
	*
	*	Change Logs:
	*	Date					Author					Notes
	*	2014-6-14				Jun.Lu					First implement
	*
	********************************************************************
	*/

/**
 * @addtogroup STM32
 */
/*@{*/
/*	Includes -------------------------------------------------------*/
#include <stdio.h>
#include "stm32f4xx.h"
#include <rtthread.h>
#include "mpu9250.h"
#include "mpu9250_app.h"
#include "DataScope_DP.h"
#include "algorithm_ahrs.h"
#include <stm32f4xx_conf.h>
/*	Global variables -----------------------------------------------*/
SENSOR_9AXIS Acc, Gyr, Mag, Ang;
float sx,sy,sz;
int16_t Temp;
float Temp_True;
extern unsigned char DataScope_OutPut_Buffer[42];
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
//因为模块的1脚没接VDDIO，所以I2C用不了，导致地磁计读不出来数，因此
//关闭这一功能
//#define USE_MAG
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/

/**
	*	@brief		mpu9250_module_read
	*	@note		read 9 axis values from mpu9250
	*	@param		None
	*	@retval		None
	*/
void mpu9250_module_read( void )
{
  uint8_t ReadBuf[24] = {0};

  MPU9250_Read(ReadBuf);

  Acc.X = (int16_t)Byte16(ReadBuf[0],  ReadBuf[1]);   // Acc_X
  Acc.Y = (int16_t)Byte16(ReadBuf[2],  ReadBuf[3]);   // Acc_Y
  Acc.Z = (int16_t)Byte16(ReadBuf[4],  ReadBuf[5]);   // Acc_Z
  Temp	= (int16_t)Byte16(ReadBuf[6],  ReadBuf[7]);   // Temp
  Gyr.X = (int16_t)Byte16(ReadBuf[8],  ReadBuf[9]);   // Gyr_X
  Gyr.Y = (int16_t)Byte16(ReadBuf[10], ReadBuf[11]);  // Gyr_Y
  Gyr.Z = (int16_t)Byte16(ReadBuf[12], ReadBuf[13]);  // Gyr_Z
 #ifdef USE_MAG
  Mag.X = (int16_t)Byte16(ReadBuf[14], ReadBuf[15]);  // Mag_X
  Mag.Y = (int16_t)Byte16(ReadBuf[17], ReadBuf[16]);  // Mag_Y
  Mag.Z = (int16_t)Byte16(ReadBuf[19], ReadBuf[18]);  // Mag_Z
#endif
}

void Data_Scope_work(void)
{
	uint8_t Send_Count,i;
	DataScope_Get_Channel_Data( AngE.Roll , 1 );
	DataScope_Get_Channel_Data( AngE.Pitch , 2 ); 
	DataScope_Get_Channel_Data( AngE.Yaw , 3 ); 
	
	DataScope_Get_Channel_Data( Acc.TrueX , 4 ); 
	DataScope_Get_Channel_Data( Acc.TrueY , 5 ); 
	DataScope_Get_Channel_Data(	Acc.TrueZ , 6 );
	DataScope_Get_Channel_Data( NumQ.q0 , 7 ); 
	DataScope_Get_Channel_Data( NumQ.q1 , 8 ); 
	DataScope_Get_Channel_Data( NumQ.q2 , 9 );
	DataScope_Get_Channel_Data( NumQ.q3 , 10 );
	Send_Count=DataScope_Data_Generate(10);
	for( i = 0 ; i < Send_Count; i++)  //循环发送,直到发送完毕   
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; //从串口丢一个字节数据出去      
	 }
}

void Kalman_Processing(uint8_t each_state[], int16_t temp[][2])
{
	Acc.Kalman_X=kalman(Acc.X,&each_state[ACC_X],&temp[ACC_X][0]);
	Acc.Kalman_Y=kalman(Acc.Y,&each_state[ACC_Y],&temp[ACC_Y][0]);
	Acc.Kalman_Z=kalman(Acc.Z,&each_state[ACC_Z],&temp[ACC_Z][0]);
	Gyr.Kalman_X=kalman(Gyr.X,&each_state[GYR_X],&temp[GYR_X][0]);
	Gyr.Kalman_Y=kalman(Gyr.Y,&each_state[GYR_Y],&temp[GYR_Y][0]);
	Gyr.Kalman_Z=kalman(Gyr.Z,&each_state[GYR_Z],&temp[GYR_Z][0]);
}

void get_true(void)
{
	Acc.TrueX = Acc.Kalman_X*MPU9250A_4g*9.8;      // g/LSB
	Acc.TrueY = Acc.Kalman_Y*MPU9250A_4g*9.8;      // g/LSB
	Acc.TrueZ = Acc.Kalman_Z*MPU9250A_4g*9.8;      // g/LSB
	Temp_True	= 21+Temp*MPU9250T_85degC;
	Gyr.TrueX = Gyr.Kalman_X*MPU9250G_2000dps; // dps/LSB
	Gyr.TrueY = Gyr.Kalman_Y*MPU9250G_2000dps; // dps/LSB
	Gyr.TrueZ = Gyr.Kalman_Z*MPU9250G_2000dps; // dps/LSB
#ifdef USE_MAG
	Mag.TrueX = Mag.X*MPU9250M_4800uT;  // uT/LSB
	Mag.TrueY = Mag.Y*MPU9250M_4800uT;  // uT/LSB
	Mag.TrueZ = Mag.Z*MPU9250M_4800uT;  // uT/LSB
#endif

	AHRS_Update();
}


ALIGN(RT_ALIGN_SIZE)
static char thread_mpu9250_app_stack[1024];
struct rt_thread thread_mpu9250_app;
static void rt_thread_entry_mpu9250_app(void* parameter)
{
    
	char f_buf_temp[10],f_buf_yaw[10],f_buf_pitch[10],f_buf_roll[10],
			true_x[10],true_y[10],true_z[10],
			true_gx[10],true_gy[10],true_gz[10],
			true_mx[10],true_my[10],true_mz[10];	
	uint8_t state[6]={0};
	int16_t kalman_temp[6][2]={0};
	uint8_t i;
	MPU9250_Config();
	rt_thread_delay(10);
	MPU9250_Init();
	AHRS_Init(&NumQ, &AngE);
	while (1)
    {
		//MPU9250_Check_MPU6500();
		//MPU9250_Check_AK8963();
		for(i=0;i<20;i++)
		{
			mpu9250_module_read();
			//-----卡尔曼滤波---------------------------
			Kalman_Processing(state, kalman_temp);
			//-----换算值---------------------------
			get_true();
		}
		//-----通过图形上位机显示数据----------------
		Data_Scope_work();
		
		//-----显示加速度-----------------------------
		/*
		rt_kprintf("Acc.X=%d\r\n",Acc.X);
		rt_kprintf("Acc.Y=%d\r\n",Acc.Y);
		rt_kprintf("Acc.Z=%d\r\n",Acc.Z);
		sprintf(true_x,"%3.3f",Acc.TrueX);
		sprintf(true_y,"%3.3f",Acc.TrueY);
		sprintf(true_z,"%3.3f",Acc.TrueZ);
		rt_kprintf("Acc.TrueX=%s\r\n",true_x);
		rt_kprintf("Acc.TrueY=%s\r\n",true_y);
		rt_kprintf("Acc.TrueZ=%s\r\n",true_z);
		*/
		//-----显示角加速度-------------------------------
		/*
		rt_kprintf("Gyr.X=%d\r\n",Gyr.X);
		rt_kprintf("Gyr.Y=%d\r\n",Gyr.Y);
		rt_kprintf("Gyr.Z=%d\r\n",Gyr.Z);
		sprintf(true_gx,"%3.3f",Gyr.TrueX);
		sprintf(true_gy,"%3.3f",Gyr.TrueY);
		sprintf(true_gz,"%3.3f",Gyr.TrueZ);
		rt_kprintf("Gyr.TrueX=%s\r\n",true_gx);
		rt_kprintf("Gyr.TrueY=%s\r\n",true_gy);
		rt_kprintf("Gyr.TrueZ=%s\r\n",true_gz);
		*/
		//-----显示磁力角----------------------------------
 #ifdef USE_MAG
		rt_kprintf("Mag.X=%d\r\n",Mag.X);
		rt_kprintf("Mag.Y=%d\r\n",Mag.Y);
		rt_kprintf("Mag.Z=%d\r\n",Mag.Z);
		sprintf(true_mx,"%3.3f",Mag.TrueX);
		sprintf(true_my,"%3.3f",Mag.TrueY);
		sprintf(true_mz,"%3.3f",Mag.TrueZ);
		rt_kprintf("Mag.TrueX=%s\r\n",true_mx);
		rt_kprintf("Mag.TrueY=%s\r\n",true_my);
		rt_kprintf("Mag.TrueZ=%s\r\n",true_mz);
#endif
		//-----显示欧拉角--------------------------------------
		/*
		sprintf(f_buf_yaw,"%3.3f",AngE.Yaw);
		sprintf(f_buf_pitch,"%3.3f",AngE.Pitch);
		sprintf(f_buf_roll,"%3.3f",AngE.Roll);
		rt_kprintf("AngE.Yaw=%s\r\n",f_buf_yaw);
		rt_kprintf("AngE.Pitch=%s\r\n",f_buf_pitch);
		rt_kprintf("AngE.Roll=%s\r\n",f_buf_roll);
		*/
		//------显示温度-----------------------------------------------
		/**/
		//sprintf(f_buf_temp,"%3.3f",Temp_True);
		//xxrt_kprintf("Temperature=%s\r\n",f_buf_temp);
		
		//------延时函数-----------------------------------------------
		rt_thread_delay(2);
    }
}


int nano_board_mpu9250_app_init(void)
{  
    rt_thread_init(&thread_mpu9250_app,
                   "mpu9250_app",
                   rt_thread_entry_mpu9250_app,
                   RT_NULL,
                   &thread_mpu9250_app_stack[0],
                   sizeof(thread_mpu9250_app_stack),12,5);
    rt_thread_startup(&thread_mpu9250_app);
    
    return 0;
}

/*@}*/
