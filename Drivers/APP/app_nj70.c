#include "app_nj70.h"
#include "app_joint.h"

uint8_t tx_buffer[MAX_TX_BUFFER_SIZE];
uint8_t rx_buffer[MAX_RX_BUFFER_SIZE];
uint32_t tx_tick = 0;
_Bool rx_flag = 0;

_Bool torque_first_flag = 0;   /* 1 = 已收到第一帧有效力矩 */
float alpha = 0.7;
float torque = 0;
float torque_after_comp = 0;

void App_NJ70_Task(void)
{
	/* ---- 1. 发：50 ms 一问一答，上一帧还没处理完就不发 ---- */
	if(uwTick - tx_tick >= 50)
	{
		tx_tick = uwTick;

		if(rx_flag == 0)
		{
			Dev_NJ70_Send_Quick_Cmd(tx_buffer);

			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
			__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
		}
	}

	/* ---- 2. 收：有新帧就解析 ---- */
	if(rx_flag == 0)
		return;
	
	if(rx_buffer[0] == 0x01 && rx_buffer[1] == 0x06) // 快捷读取传感器数据指令
	{
		uint8_t sum_value = Com_AddCheck(rx_buffer,QUICK_CMD_SIZE-1);
		uint8_t sum_from_rx = rx_buffer[QUICK_CMD_SIZE-1];
		if(sum_value == sum_from_rx)
		{
			uint32_t torque_org = (rx_buffer[QUICK_CMD_SIZE-5] << 24) | \
								  (rx_buffer[QUICK_CMD_SIZE-4] << 16) | \
								  (rx_buffer[QUICK_CMD_SIZE-3] << 8)  | \
								  (rx_buffer[QUICK_CMD_SIZE-2] << 0);
			float torque_temp = *(float*)&torque_org;
			if(torque_first_flag == 1)
			{
				torque = Com_LowPass_Filter(alpha,torque_temp,torque);
			}
			else
			{
				torque = torque_temp;   /* 首帧直接给初值，避免从 0 爬上来 */
				torque_first_flag = 1;
			}
		}
	}
	
	int32_t pos = 0;
	if(Dev_Joint_ReadPos(&pos))
	{
		float tau_g;
		tau_g = App_Joint_GetGravityTorque(pos);
		torque_after_comp   = torque - tau_g;
	}
	
	memset(rx_buffer,0,sizeof(rx_buffer));
	rx_flag = 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
		rx_flag = 1;
    }
}

