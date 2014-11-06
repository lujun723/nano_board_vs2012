/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include "stm32f4xx.h"
#include <rtthread.h>
#include "bluetooth_4_0.h"

uint8_t ble_tx_buff[120]={"Hello,BLE~!\r\n"},
ble_rx_buff[120]={0};
static rt_mq_t ble_rx_mq;



struct rx_msg
{
    rt_device_t dev;
    rt_size_t size;
};

rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    msg.dev = dev;
    msg.size = size;
    
    rt_mq_send(ble_rx_mq, &msg, sizeof(struct rx_msg));
    
    return RT_EOK;
}


void uart_sync_show(rt_device_t uart_sync_device,struct rx_msg read_msg,uint8_t *read_buffer)
{
    uint8_t length_of_read;
    
    length_of_read = (sizeof(read_buffer) - 1) >
    read_msg.size ? read_msg.size:sizeof(read_buffer)-1;
    
    length_of_read = rt_device_read(read_msg.dev, 0,
                                    &ble_rx_buff[0], length_of_read);
    //uart_rx_buffer[rx_length] = '\0';
    if(uart_sync_device != RT_NULL)
    {
        rt_device_write(uart_sync_device, 0, read_buffer, length_of_read);
        //rt_kprintf("get some\r\n");
    }
}

struct CMD_DATA_UPPACK
{
    uint8_t *data;
    uint8_t uppack_state;
    uint8_t content;
    uint8_t sum;
};

struct CMD_DATA_UPPACK read_cmd;

uint8_t (*cmd_func)(uint8_t  cmd_content);

/*void cmd_unpack(uint8_t *read_buffer)
{
    read_cmd->data=read_buffer;
    switch(read_cmd->uppack_state)
    {
        case WAIT:
            if(read_cmd->*data==0x06){
                read_cmd->uppack_state=KIND;
                rt_kprintf("in wait statement\r\n");
            }
            break;
        case KIND:
            read_cmd->uppack_state=CONTENT;
            switch (read_cmd->*data) {
                case ON/OFF:
                    cmd_func=on_off_ctrl;//函数指针赋值
                    read_cmd->sum=ON/OFF;
                    rt_kprintf("in kind on/off statement\r\n");
                    break;
                case GEAR:
                    cmd_func=gear_ctrl;
                    read_cmd->sum=GEAR;
                    rt_kprintf("in kind gear statement\r\n");
                    break;
                case ACC:
                    cmd_func=acc_ctrl;
                    read_cmd->sum=ACC;
                    rt_kprintf("in kind accelerator statement\r\n");
                    break;
                case DIR:
                    cmd_func=dir_ctrl;
                    read_cmd->sum=DIR;
                    rt_kprintf("in kind direction statement\r\n");
                    break;
                default:
                    read_cmd->sum=CLEAR;
                    read_cmd->uppack_state=WAIT;
                    break;
            }
        case CONTENT:
            read_cmd->content=read_cmd->*data;
            read_cmd->uppack_state=CHECK;
            rt_kprintf("in content statement\r\n");
            break;
        case CHECK:
            read_cmd->uppack_state=END;
            if (read_cmd->sum==read_cmd->*data) {
                cmd_func(read_cmd->*data);
            }
            else{
                read_cmd->sum=CLEAR;
                read_cmd->content=CLEAR;
                read_cmd->uppack_state=WAIT;
            }
            break;
        case END:
            read_cmd->sum=CLEAR;
            read_cmd->content=CLEAR;
            read_cmd->uppack_state=WAIT;
            break;
        default:
            break;

    }
}
*/

ALIGN(RT_ALIGN_SIZE)
static char thread_ble_stack[1024];
struct rt_thread thread_ble;
static void rt_thread_entry_ble(void* parameter)
{
    rt_device_t ble_device,ble_write_device;
    
    rt_err_t result;
    uint8_t tx_length=0,rx_length=0;
    struct rx_msg ble_rx_msg;

    ble_device = rt_device_find("uart3");
	ble_write_device = ble_device;
    if (ble_device!= RT_NULL)
    {
        
        rt_device_set_rx_indicate(ble_device, uart_input);
        rt_device_open(ble_device, RT_DEVICE_OFLAG_RDWR);
    }
    tx_length =sizeof(ble_tx_buff);
    while (1)
    {
        result = rt_mq_recv(ble_rx_mq, &ble_rx_msg,
                            sizeof(struct rx_msg), 5);
        
        if (result == RT_EOK)
        {
            uart_sync_show(ble_write_device,ble_rx_msg,ble_rx_buff);
        }
		//rt_kprintf("test\r\n");
		//rt_thread_delay(100);
        
    }
}

static struct rt_messagequeue s_your_mq;
static char s_msg_pool[1024];
int nano_board_ble_init(void)
{
    
    rt_mq_init(&s_your_mq,"ble_buff",&s_msg_pool[0],
               sizeof(struct rx_msg),sizeof(s_msg_pool),
               RT_IPC_FLAG_FIFO);
    ble_rx_mq = &s_your_mq;
    
    rt_thread_init(&thread_ble,
                   "ble",
                   rt_thread_entry_ble,
                   RT_NULL,
                   &thread_ble_stack[0],
                   sizeof(thread_ble_stack),10,5);
    rt_thread_startup(&thread_ble);
    
    return 0;
}

/*@}*/
