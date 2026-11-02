#include "app_nj70.h"
#include "app_joint.h"

#define NJ70_PERIOD_MS      50u     /* 20 Hz 一问一答 */
#define NJ70_LPF_ALPHA      0.7f    /* 一阶低通系数，20 Hz 采样下截止约 2.5 Hz */

uint8_t tx_buffer[MAX_TX_BUFFER_SIZE];
uint8_t rx_buffer[MAX_RX_BUFFER_SIZE];
_Bool   torque_first_flag = 0;      /* 1 = 已收到第一帧有效力矩 */
float   torque = 0.0f;
float   torque_after_comp = 0.0f;

static uint32_t s_tx_tick = 0;
static _Bool    s_rx_flag = 0;      /* 接收中断置 1，解析完清 0 */

void App_NJ70_Task(void)
{
	/* ---- 1. 发：上一帧还没收回来就不发下一帧 ---- */
	if (uwTick - s_tx_tick >= NJ70_PERIOD_MS)
	{
		s_tx_tick = uwTick;

		if (s_rx_flag == 0)
		{
			Dev_NJ70_Send_Quick_Cmd(tx_buffer);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
			__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
		}
	}

	/* ---- 2. 收：有新帧才解析 ---- */
	if (s_rx_flag == 0)
	{
		return;
	}

	if (rx_buffer[0] == 0x01 && rx_buffer[1] == 0x06)
	{
		uint8_t sum_value  = Com_AddCheck(rx_buffer, QUICK_CMD_SIZE - 1);
		uint8_t sum_from_rx = rx_buffer[QUICK_CMD_SIZE - 1];

		if (sum_value == sum_from_rx)
		{
			/* 力矩是 float 大端，落在第 3~6 字节 */
			uint32_t torque_org = ((uint32_t)rx_buffer[QUICK_CMD_SIZE - 5] << 24) |
			                      ((uint32_t)rx_buffer[QUICK_CMD_SIZE - 4] << 16) |
			                      ((uint32_t)rx_buffer[QUICK_CMD_SIZE - 3] << 8)  |
			                      ((uint32_t)rx_buffer[QUICK_CMD_SIZE - 2]);
			float torque_temp = *(float *)&torque_org;

			if (torque_first_flag == 1)
			{
				torque = Com_LowPass_Filter(NJ70_LPF_ALPHA, torque_temp, torque);
			}
			else
			{
				torque = torque_temp;       /* 首帧直接给初值，避免从 0 爬上来 */
				torque_first_flag = 1;
			}
		}
	}

	/* ---- 3. 扣重力矩：导纳环真正吃的是"外力" ---- */
	int32_t pos = 0;
	if (Dev_Joint_ReadPos(&pos))
	{
		torque_after_comp = torque - App_Joint_GetGravityTorque(pos);
	}

	memset(rx_buffer, 0, sizeof(rx_buffer));
	s_rx_flag = 0;
}

/* UART IDLE + DMA 收完一帧，在中断里只置标志 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	(void)Size;

	if (huart->Instance == USART2)
	{
		s_rx_flag = 1;
	}
}
