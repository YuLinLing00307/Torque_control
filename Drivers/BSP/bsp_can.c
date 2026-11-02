#include "bsp_can.h"

/* CAN1：32 位掩码模式，ID 和掩码全 0 = 全部放行（单节点总线不需要过滤） */
void Bsp_Can_Init(void)
{
	CAN_FilterTypeDef sFilterConfig = {0};

	sFilterConfig.FilterBank           = 0;
	sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh         = 0x0000;
	sFilterConfig.FilterIdLow          = 0x0000;
	sFilterConfig.FilterMaskIdHigh     = 0x0000;   /* 掩码位 0 = 该位不关心 */
	sFilterConfig.FilterMaskIdLow      = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation     = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;       /* CAN2 起始 bank；本工程不用 CAN2，仅占位 */

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		printf("CAN1 过滤器初始化失败！\r\n");
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		printf("CAN1 接收中断使能失败！\r\n");
	}

	HAL_CAN_Start(&hcan);
}

/* 发一帧标准帧或扩展帧。注意本函数会被 TIM3 中断调用。 */
uint8_t Bsp_Can_SendData(uint32_t type, uint32_t ID, uint8_t *msgData, uint8_t num)
{
	/* 必须用局部变量：static 会被 TIM3 中断和主循环同时写，互相踩 */
	CAN_TxHeaderTypeDef canTxHeader = {0};
	uint32_t txMailbox = 0;
	volatile uint32_t guard = 20000u;

	if (num > 8)
	{
		num = 8;
	}

	if (type == CAN_ID_EXT)
	{
		canTxHeader.ExtId = ID;
		canTxHeader.IDE   = CAN_ID_EXT;
	}
	else
	{
		canTxHeader.StdId = ID;
		canTxHeader.IDE   = CAN_ID_STD;
	}
	canTxHeader.RTR = CAN_RTR_DATA;
	canTxHeader.DLC = num;
	canTxHeader.TransmitGlobalTime = DISABLE;

	/* 等邮箱空。不能用 HAL_Delay：TIM3 抢占优先级(1) 高于 SysTick(15)，
	 * 中断里 uwTick 不涨，HAL_Delay 永远返回不了，整机会卡死在这里。 */
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
		if (--guard == 0u)
		{
			return 3;
		}
	}

	return HAL_CAN_AddTxMessage(&hcan, &canTxHeader, msgData, &txMailbox);
}
