/**
  ******************************************************************************
  * @�ļ���     �� bsp_usart.c
  * @����       �� strongerHuang
  * @�汾       �� V1.0.0
  * @����       �� 2018��12��18��
  * @ժҪ       �� USART�ײ�����Դ�ļ�
  ******************************************************************************/
/*----------------------------------------------------------------------------
  ������־:
  2018-12-18 V1.0.0:��ʼ�汾
  ----------------------------------------------------------------------------*/
/* ������ͷ�ļ� --------------------------------------------------------------*/
#include "bsp_usart.h"


/***************************************** DEBUG���Դ��� *****************************************/
/************************************************
�������� �� UART_DEBUG_Configuration
��    �� �� ���Դ�������
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void UART_DEBUG_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* ʱ������ */
  RCC_APB2PeriphClockCmd(DEBUG_USART_TX_GPIO_CLK | DEBUG_USART_RX_GPIO_CLK , ENABLE);

  if(USART1 == DEBUG_USARTx)
    RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
  else
    RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

  /* �������� */
  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;                    //UART Tx
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;                    //UART Rx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* NVIC���� */
  NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = DEBUG_USART_Priority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* UART���� */
  USART_InitStructure.USART_BaudRate = DEBUG_USART_BaudRate;           //������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;        //����λ��
  USART_InitStructure.USART_StopBits = USART_StopBits_1;             //ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No ;               //У��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    //�շ�����
  USART_Init(DEBUG_USARTx, &USART_InitStructure);

  USART_ClearFlag(DEBUG_USARTx, USART_FLAG_RXNE | USART_FLAG_TC);
  USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);                  //�����ж�

  USART_Cmd(DEBUG_USARTx, ENABLE);                                      //ʹ��UART
}

/************************************************
�������� �� DEBUG_SendByte
��    �� �� ���Դ��ڷ���һ���ֽ�
��    �� �� Data --- ����
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void DEBUG_SendByte(uint8_t Data)
{
  while(RESET == USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE));
  USART_SendData(DEBUG_USARTx, Data);
}

/************************************************
�������� �� DEBUG_SendNByte
��    �� �� ���Դ��ڷ���N���ֽ�
��    �� �� pData ---- ����
            Length --- ����
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void DEBUG_SendNByte(uint8_t *pData, uint16_t Length)
{
  while(Length--)
  {
    DEBUG_SendByte(*pData);
    pData++;
  }
}

/************************************************
�������� �� fputc
��    �� �� �ض���putc����
��    �� �� ch --- �ַ�
            *f --- �ļ�ָ��
�� �� ֵ �� �ַ�
��    �� �� strongerHuang
*************************************************/
int fputc(int ch, FILE *f)
{
  DEBUG_SendByte((uint8_t)ch);

  return ch;
}

/************************************************
�������� �� fgetc
��    �� �� �ض���getc����
��    �� �� *f --- �ļ�ָ��
�� �� ֵ �� �ַ�
��    �� �� strongerHuang
*************************************************/
int fgetc(FILE *f)
{
  while(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE) == RESET);

  return (int)USART_ReceiveData(DEBUG_USARTx);
}


/************************************************
�������� �� USART_Initializes
��    �� �� ���ڳ�ʼ��
��    �� �� ��
�� �� ֵ �� ��
��    �� �� strongerHuang
*************************************************/
void USART_Initializes(void)
{
  UART_DEBUG_Configuration();
}


/**** Copyright (C)2018 strongerHuang. All Rights Reserved **** END OF FILE ****/
