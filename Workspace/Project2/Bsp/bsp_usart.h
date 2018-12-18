/**
  ******************************************************************************
  * @文件名     ： bsp_usart.h
  * @作者       ： strongerHuang
  * @版本       ： V1.0.0
  * @日期       ： 2018年12月18日
  * @摘要       ： USART底层驱动头文件
  ******************************************************************************/

/* 定义防止递归包含 ----------------------------------------------------------*/
#ifndef _BSP_USART_H
#define _BSP_USART_H

/* 包含的头文件 --------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>


/* 宏定义 --------------------------------------------------------------------*/
/* DEBUG */
#define DEBUG_USARTx               USART1
#define DEBUG_USART_CLK            RCC_APB2Periph_USART1
#define DEBUG_USART_TX_GPIO_CLK    RCC_APB2Periph_GPIOA     //USART TX
#define DEBUG_USART_TX_PIN         GPIO_Pin_9
#define DEBUG_USART_TX_GPIO_PORT   GPIOA
#define DEBUG_USART_RX_GPIO_CLK    RCC_APB2Periph_GPIOA     //USART RX
#define DEBUG_USART_RX_PIN         GPIO_Pin_10
#define DEBUG_USART_RX_GPIO_PORT   GPIOA
#define DEBUG_USART_IRQn           USART1_IRQn
#define DEBUG_USART_Priority       10                       //优先级
#define DEBUG_USART_BaudRate       115200                   //波特率
#define DEBUG_USART_IRQHandler     USART1_IRQHandler        //中断函数接口(见stm32f10x_it.c)


/* 函数申明 ------------------------------------------------------------------*/
void DEBUG_SendByte(uint8_t Data);
void DEBUG_SendNByte(uint8_t *pData, uint16_t Length);

void USART_Initializes(void);


#endif /* _BSP_USART_H */

/**** Copyright (C)2018 strongerHuang. All Rights Reserved **** END OF FILE ****/
