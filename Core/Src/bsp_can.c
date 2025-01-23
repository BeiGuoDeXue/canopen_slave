/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TXHeader;
CAN_RxHeaderTypeDef RxHeader;
///CAN接收回调函数
static Message RxMSG;
static uint32_t pTxMailbox = 0;

uint8_t rxbuf[8];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Configure_Filter(void)
{
    CAN_FilterTypeDef sFilterConfig;
    
    sFilterConfig.FilterIdHigh=0X0000;     //32?ID
    sFilterConfig.FilterIdLow=0X0000;
    sFilterConfig.FilterMaskIdHigh=0X0000; //32?MASK
    sFilterConfig.FilterMaskIdLow=0X0000;  
    sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;//������0������FIFO0
    sFilterConfig.FilterBank=1;          //������0
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation=ENABLE; //�����??���?0
    sFilterConfig.SlaveStartFilterBank=14;
    
    if(HAL_CAN_ConfigFilter(&hcan1,&sFilterConfig)!=HAL_OK)
    {
        Error_Handler();
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  uint8_t rxbuf[8];
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, rxbuf) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  RxMSG.cob_id = (uint16_t)RxHeader.StdId;
  if (RxHeader.RTR == CAN_RTR_REMOTE)
  {
    RxMSG.rtr = 1;
  }
  else
  {
    RxMSG.rtr = 0;
  }

  RxMSG.len = RxHeader.DLC;
  printf("RxMSG.cob_id = %04X\r\n", RxMSG.cob_id);
  printf("RxMSG.rtr = %d\r\n", RxMSG.rtr);
  printf("RxMSG.len = %d\r\n", RxMSG.len);
  for(int i = 0; i < RxMSG.len; i++)
  {
    RxMSG.data[i] = rxbuf[i];
    printf("RxMSG.data[%d] = %02X\r\n", i, RxMSG.data[i]);
  }
  canDispatch(&TestAllSlave_Data, &(RxMSG));
}


/*
 * 函数名：canSend
 * 描述  ：CAN发送函数
 * 输入  ：发送报文结构体
 * 输出  : 无
 * 调用  ：外部调用
 */
unsigned char canSend(CAN_PORT notused, Message *msg)
{
  TXHeader.StdId = msg->cob_id;
  TXHeader.DLC = msg->len;
  TXHeader.IDE = CAN_ID_STD;
  if(msg->rtr)
    TXHeader.RTR = CAN_RTR_REMOTE;
  else
  TXHeader.RTR = CAN_RTR_DATA;
  TXHeader.TransmitGlobalTime = DISABLE;
  uint8_t TXmessage[8];
  for(int i = 0; i < msg->len; i++)
    TXmessage[i] = msg->data[i];
  if(HAL_CAN_AddTxMessage(&hcan1,&TXHeader,TXmessage,&pTxMailbox) == HAL_OK)
    return 0;
  else
    return 1;
}


/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
