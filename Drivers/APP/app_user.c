#include "app_user.h"
#include "app_joint.h"

uint32_t led_tick = 0;
void App_Led_Task(void)
{
	if(uwTick - led_tick < 1000)
		return;
	led_tick = uwTick;
	
	HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
}

uint32_t key_value;
uint32_t key_value_old;
uint32_t key_down,key_up;
static void Key_read(void)
{
	if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin) == GPIO_PIN_RESET)
		key_value = 1;
	else
		key_value = 0;
	
	key_down = key_value  & (key_value ^ key_value_old);
	key_up   = ~key_value & (key_value ^ key_value_old);
	key_value_old = key_value;
}

uint32_t key_tick = 0;
void App_Key_Task(void)
{
	if(uwTick - key_tick < 20)
		return;
	key_tick = uwTick;
	Key_read();
	
	/* KEY0：按一下启动导纳环，再按一下停机。
	 * 启动是 20 ms 一步的非阻塞状态机，整个过程 LED1 保持常亮=运行中。*/
	if(key_down == 1)
	{
		if(App_Joint_Adm_IsBusy() != 0u)
			App_Joint_Adm_StopKey();
		else
			App_Joint_Adm_Start();
	}

	if(App_Joint_Adm_IsBusy() != 0u)
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);   /* 亮 */
	else
		HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);     /* 灭 */
}
