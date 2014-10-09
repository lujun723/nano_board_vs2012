/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author      Notes
 * 2009-01-05     Bernard     the first version
 * 2010-03-29     Bernard     remove interrupt Tx and DMA Rx mode
 * 2012-02-08     aozima      update for F4.
 * 2013-02-19			Jun.Lu			modify for F4 usart3.
 */

#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include <stm32f4xx_conf.h>

#ifdef RT_USING_UART1
struct stm32_serial_int_rx uart1_int_rx;
struct stm32_serial_device uart1 =
{
	USART1,
	&uart1_int_rx,
	RT_NULL
	//&uart1_dma_tx
};
struct rt_device uart1_device;
#endif

#ifdef RT_USING_UART2
struct stm32_serial_int_rx uart2_int_rx;
struct stm32_serial_device uart2 =
{
	USART2,
	&uart2_int_rx,
	RT_NULL
};
struct rt_device uart2_device;
#endif

#ifdef RT_USING_UART3
struct stm32_serial_int_rx uart3_int_rx;
struct stm32_serial_device uart3 =
{
	USART3,
	&uart3_int_rx,
	RT_NULL
};
struct rt_device uart3_device;
#endif


/* USART1_REMAP = 0 */

#define UART1_GPIO_TX			GPIO_Pin_9
#define UART1_TX_PIN_SOURCE 	GPIO_PinSource9
#define UART1_GPIO_RX			GPIO_Pin_10
#define UART1_RX_PIN_SOURCE 	GPIO_PinSource10
#define UART1_GPIO				GPIOA
#define UART1_GPIO_RCC      	RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART1		RCC_APB2Periph_USART1

/**/
#define UART2_GPIO_TX	    	GPIO_Pin_2
#define UART2_TX_PIN_SOURCE 	GPIO_PinSource2
#define UART2_GPIO_RX	    	GPIO_Pin_3
#define UART2_RX_PIN_SOURCE 	GPIO_PinSource3
#define UART2_GPIO	    		GPIOA
#define UART2_GPIO_RCC   		RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART2		RCC_APB1Periph_USART2

//USART3_REMAP[1:0] = 00 

#define UART3_GPIO_TX			GPIO_Pin_10
#define UART3_TX_PIN_SOURCE 	GPIO_PinSource10
#define UART3_GPIO_RX			GPIO_Pin_11
#define UART3_RX_PIN_SOURCE 	GPIO_PinSource11
#define UART3_GPIO				GPIOB
#define UART3_GPIO_RCC   		RCC_AHB1Periph_GPIOB
#define RCC_APBPeriph_UART3		RCC_APB1Periph_USART3



static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
	/* Enable USART1 GPIO clocks */
	RCC_AHB1PeriphClockCmd(UART1_GPIO_RCC, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	/* Enable USART2 clock */
	RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART1, ENABLE);
#endif

#ifdef RT_USING_UART2
	/* Enable USART2 GPIO clocks */
	RCC_AHB1PeriphClockCmd(UART2_GPIO_RCC, ENABLE);//
	/* Enable USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART2, ENABLE);//
#endif

#ifdef RT_USING_UART3 
#endif
	/* Enable USART3 GPIO clocks */
	RCC_AHB1PeriphClockCmd(UART3_GPIO_RCC, ENABLE);
	/* Enable USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APBPeriph_UART3, ENABLE);


}

static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	

#ifdef RT_USING_UART1
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	/* Configure USART2 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX | UART1_GPIO_RX;
	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

	/* Connect alternate function */
	GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PIN_SOURCE, GPIO_AF_USART1);
	GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PIN_SOURCE, GPIO_AF_USART1);
#endif

#ifdef RT_USING_UART2
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	/* Configure USART2 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX | UART2_GPIO_RX;
	GPIO_Init(UART2_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART2_GPIO, UART2_TX_PIN_SOURCE, GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_GPIO, UART2_RX_PIN_SOURCE, GPIO_AF_USART2);
#endif

#ifdef RT_USING_UART3
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	/* Configure USART3 Rx/tx PIN */
	GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX | UART3_GPIO_RX;
	GPIO_Init(UART3_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART3_GPIO, UART3_TX_PIN_SOURCE, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_GPIO, UART3_RX_PIN_SOURCE, GPIO_AF_USART3);
#endif
}

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef RT_USING_UART1
	/* Enable the USART1 Interrupt */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif

#ifdef RT_USING_UART2
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#ifdef RT_USING_UART3
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

#endif
}


volatile USART_TypeDef * uart3_debug = USART3;
/*
 * Init all related hardware in here
 * rt_hw_serial_init() will register all supported USART device
 */
void rt_hw_usart_init()
{
	USART_InitTypeDef USART_InitStructure;

	RCC_Configuration();

	GPIO_Configuration();

	NVIC_Configuration();

	/* uart init */
#ifdef RT_USING_UART1
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* register uart1 */
	rt_hw_serial_register(&uart1_device, "uart1",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX |RT_DEVICE_FLAG_STREAM,
		&uart1);

	/* enable interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART2
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	/* register uart2 */
	rt_hw_serial_register(&uart2_device, "uart2",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
		&uart2);

	/* Enable USART2 DMA Rx request */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
#endif

#ifdef RT_USING_UART3
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	/* register uart3 */
	rt_hw_serial_register(&uart3_device, "uart3",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
		&uart3);

	/* Enable USART3 DMA Rx request */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

#endif
}
