#include "app_user.h"
#include "app_joint.h"

#define LED_PERIOD_MS   1000u   /* LED0 心跳 */
#define KEY_PERIOD_MS   20u     /* 按键扫描周期 */

/* 按键状态：key_value 当前键值，key_down 本拍按下沿 */
static uint32_t led_tick      = 0;
static uint32_t key_tick      = 0;
static uint32_t key_value     = 0;
static uint32_t key_value_old = 0;
static uint32_t key_down      = 0;

void App_Led_Task(void)
{
	if (uwTick - led_tick < LED_PERIOD_MS)
	{
		return;
	}
	led_tick = uwTick;

	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
}

/* 读当前按下的键：0 无 / 1 KEY0 / 2 KEY1，并算出按下沿 */
static void Key_read(void)
{
	if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
	{
		key_value = 1;
	}
	else if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
	{
		key_value = 2;
	}
	else
	{
		key_value = 0;
	}

	key_down      = key_value & (key_value ^ key_value_old);
	key_value_old = key_value;
}

void App_Key_Task(void)
{
	if (uwTick - key_tick < KEY_PERIOD_MS)
	{
		return;
	}
	key_tick = uwTick;
	Key_read();

	/* KEY0：按一下启动导纳环，再按一下停机。
	 * 启动是 20 ms 一步的非阻塞状态机，整个过程 LED1 常亮 = 运行中。 */
	if (key_down == 1)
	{
		if (App_Joint_Adm_IsBusy() != 0u)
		{
			App_Joint_Adm_StopKey();
		}
		else
		{
			App_Joint_Adm_Start();
		}
	}

	/* KEY1：循环切换导纳增益档（中 → 软 → 硬），用于 A/B 对比手感。
	 * 三档 ωn、ζ 都不变，只改 K，所以切的就是"推起来多硬"。
	 * 运行中也能切：位置连续、力矩有阶跃，手感差异最明显。 */
	if (key_down == 2)
	{
		App_Joint_Adm_CycleGain();
	}

	/* LED1 常亮 = 导纳环运行中 */
	if (App_Joint_Adm_IsBusy() != 0u)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	}
}
