#include "app_nj70.h"
#include "app_joint.h"

/* =====================================================================
 *  采样周期与低通系数（两者是耦合的，不能单独改）
 *
 *  一阶 IIR:  y = a*x + (1-a)*y
 *  时间常数  tau = -T / ln(1-a)        T = 采样周期
 *  截止频率  fc  = 1 / (2*pi*tau)
 *
 *  NJ70_LPF_ALPHA = 0.7 时：
 *      T = 50 ms  ->  tau = 41.5 ms,  fc = 3.8 Hz   <- 原配置
 *      T = 10 ms  ->  tau =  8.3 ms,  fc = 19 Hz    <- 延迟少 33 ms
 *  也就是说：周期从 50 改 10 时，若 alpha 不动，滤波会跟着变宽 5 倍。
 *  想保持原滤波特性必须同时降 alpha（见下表），但那等于放弃了减延迟：
 *      T = 10 ms / a = 0.21  ->  tau = 41.5 ms（等于没提速）
 *      T = 10 ms / a = 0.50  ->  tau = 14.4 ms,  fc = 11 Hz
 *
 *  当前默认 NJ70_FAST = 0，编译结果与改动前逐字节一致。
 * ===================================================================== */

#define NJ70_FAST           0u      /* 1 = 100 Hz 采样（力控推荐），0 = 20 Hz */

#if (NJ70_FAST != 0u)
#define NJ70_PERIOD_MS      10u     /* 100 Hz 一问一答，往返 0.78 ms 够用 */
#else
#define NJ70_PERIOD_MS      50u     /* 20 Hz 一问一答 */
#endif

#define NJ70_LPF_ALPHA      0.7f    /* 一阶低通系数（tau 随 T 变，见上表） */

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
		//printf("%.2f\r\n",torque_after_comp);
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
