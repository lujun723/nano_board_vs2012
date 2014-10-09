/**
	********************************************************************
	*	@file		mpu9250.c
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
#include <rtthread.h>
#include "mpu9250.h"
#include "stm32f4xx_spi.h"

/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/



/**
	*	@brief		MPU9250_Config
	*	@note		MPU9250 Init
	*	@param		None
	*	@retval		None
	*/
void MPU9250_Config( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	/* SPI Clk Init ************************************************************/
	RCC_AHB1PeriphClockCmd(SPIx_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(SPIx_CLK, ENABLE);

	GPIO_InitStruct.GPIO_Pin = SPIx_CSM_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = SPIx_SCK_PIN|SPIx_SDO_PIN|SPIx_SDI_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(SPIx_GPIO_PORT, &GPIO_InitStruct);

	GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_SCK_SOURCE, SPIx_SCK_AF);
	GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_SDO_SOURCE, SPIx_SDO_AF);
	GPIO_PinAFConfig(SPIx_GPIO_PORT, SPIx_SDI_SOURCE, SPIx_SDI_AF);
	
	SPIx_CSM_PIN_H();

	/* SPI Init ****************************************************************/
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;   // Full Duplex
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;                        // Master Mode
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;                    // Data Size 8 bit
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;                          // Transitioned On The Falling Edge
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;                         // Latched On the Rising Edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;                            // Software NSS Signal
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;   // fsck = APB2 84MHz / 4 = 21MHz
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;                   // MSB First
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPIx, &SPI_InitStruct);

	SPI_Cmd(SPIx, ENABLE);
}

/**
	*	@brief		SPI_WriteByte
	*	@note		write one byte to spi bus
	*	@param		None
	*	@retval		None
	*/
void SPI_WriteByte( SPI_TypeDef* SPI_x, uint8_t WriteByte )
{
  while((SPI_x->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI_x->DR = WriteByte;
  while((SPI_x->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
  SPI_x->DR;
}

/**
	*	@brief		SPI_ReadByte
	*	@note		write one byte to spi bus
	*	@param		None
	*	@retval		None
	*/
uint8_t SPI_ReadByte( SPI_TypeDef* SPI_x )
{
  while((SPI_x->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI_x->DR = 0xFF;
  while((SPI_x->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);

  return SPIx->DR;
}


/**
	*	@brief		MPU9250_WriteReg
	*	@note		MPU9250 write one byte
	*	@param		WriteAddr= reg to write
					WriteData= data to write
	*	@retval		None
	*/
static void MPU9250_WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
	SPIx_CSM_PIN_L();
	SPI_WriteByte(SPIx, WriteAddr);
	SPI_WriteByte(SPIx, WriteData);
	SPIx_CSM_PIN_H();
}

/**
	*	@brief		MPU9250_ReadReg
	*	@note		MPU9250 read one byte
	*	@param		ReadAddr=reg address
					ReadData=save the data of reading
	*	@retval		None
	*/
static void MPU9250_ReadReg( uint8_t ReadAddr, uint8_t *ReadData )
{
	SPIx_CSM_PIN_L();
	SPI_WriteByte(SPIx, 0x80 | ReadAddr);
	*ReadData = SPI_ReadByte(SPIx);
	SPIx_CSM_PIN_H();
}

/**
	*	@brief		MPU9250_ReadRegs
	*	@note		MPU9250 read bytes
	*	@param		ReadAddr=reg address
					ReadBuf=Buffer of read data
					Bytes=length of data to read
	*	@retval		None
	*/
static void MPU9250_ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes )
{
	uint8_t i = 0;

	SPIx_CSM_PIN_L();
	SPI_WriteByte(SPIx, 0x80 | ReadAddr);
	for(i=0; i<Bytes; i++)
	{
		ReadBuf[i] = SPI_ReadByte(SPIx);
	}
	SPIx_CSM_PIN_H();
}

/**
	*	@brief		MPU9250_Check_MPU6500
	*	@note		MPU6500 in MPU9250 ID check
	*	@param		None
	*	@retval		None
	*/
uint8_t MPU9250_Check_MPU6500( void )
{
	uint8_t DeviceID = 0x00;

	/* MPU6500 */
	DeviceID = 0x00;
	MPU9250_ReadReg(MPU6500_WHO_AM_I, &DeviceID);
	rt_kprintf("MPU6500_ID=%d\r\n",DeviceID);
	if(DeviceID != MPU6500_Device_ID)
	return ERROR;
	
	return SUCCESS;
}

/**
	*	@brief		MPU9250_Check_AK8963
	*	@note		AK8963 in MPU9250 ID check
	*	@param		None
	*	@retval		None
	*/
uint8_t MPU9250_Check_AK8963( void )
{
	uint8_t DeviceID2 = 0x00;

	DeviceID2 = 0x00;
	MPU9250_WriteReg(MPU6500_I2C_SLV0_ADDR, 0x8C);          // Set AK8963_I2C_ADDR = 7'b000_1100
	rt_thread_delay(1);//Delay 10ms
	MPU9250_WriteReg(MPU6500_I2C_SLV0_REG, AK8963_WIA);     // Set Write Reg
	MPU9250_WriteReg(MPU6500_I2C_SLV0_CTRL, 0x81);          // Start Read
	rt_thread_delay(1);//Delay 10ms
	MPU9250_ReadReg(MPU6500_EXT_SENS_DATA_00, &DeviceID2);   // Read Data
	rt_kprintf("AK8963_ID=%d\r\n",DeviceID2);
	if(DeviceID2 != AK8963_Device_ID)
	return ERROR;

	return SUCCESS;
}

/**
	*	@brief		MPU9250_Read
	*	@note		MPU9250 Read Sensors' Values
	*	@param		None
	*	@retval		None
	*/
void MPU9250_Read( uint8_t *ReadBuf )
{
	MPU9250_ReadRegs(MPU6500_ACCEL_XOUT_H, ReadBuf, BYTES_TO_READ);
}

/**
	*	@brief		MPU9250_Init
	*	@note		MPU9250 module init
	*	@param		None
	*	@retval		None
	*/
void MPU9250_Init( void )
{
  uint8_t i = 0;
  uint8_t MPU6500_Init_Data[MPU9250_InitRegNum][2] = {
      {0x80, MPU6500_PWR_MGMT_1},     // Reset Device
      {0x01, MPU6500_PWR_MGMT_1},     // Clock Source
      {0x00, MPU6500_PWR_MGMT_2},     // Enable Acc & Gyro
      {0x07, MPU6500_CONFIG},         // 
      {0x18, MPU6500_GYRO_CONFIG},    // +-2000dps
      {0x08, MPU6500_ACCEL_CONFIG},   // +-4G
      {0x00, MPU6500_ACCEL_CONFIG_2}, // Set Acc Data Rates
      {0x30, MPU6500_INT_PIN_CFG},    // 
      {0x40, MPU6500_I2C_MST_CTRL},   // I2C Speed 348 kHz
      {0x20, MPU6500_USER_CTRL},      // Enable AUX
	  
	  // Set Slave to Read AK8963
      {0x18, MPU6500_I2C_SLV0_ADDR},  // AK8963_I2C_ADDR ( 7'b000_1100 )
      {0x00, MPU6500_I2C_SLV0_REG},   // AK8963_WIA ( 0x00 )
      {0x81, MPU6500_I2C_SLV0_CTRL},  // Enable
      {0x01, MPU6500_I2C_MST_DELAY_CTRL}
    };

  for(i=0; i<MPU9250_InitRegNum; i++) {
    MPU9250_WriteReg(MPU6500_Init_Data[i][1], MPU6500_Init_Data[i][0]);
    rt_thread_delay(1);//Delay 10ms
  }
}

/*@}*/
