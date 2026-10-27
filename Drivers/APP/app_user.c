#include "app_user.h"
#include "joint.h"
#include <math.h>

uint32_t led_tick = 0;
void led_proc(void)
{
	if(uwTick - led_tick < 1000)
		return;
	led_tick = uwTick;
	
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}

uint8_t tx_buffer[MAX_TX_BUFFER_SIZE];
uint32_t tx_tick = 0;
_Bool rx_flag = 0;
uint32_t rx_overtime = 0;
uint8_t rx_buffer[MAX_RX_BUFFER_SIZE];
void tx_proc(void)
{
	if(uwTick - tx_tick < 50)
		return;
	tx_tick = uwTick;
	
	if(rx_flag == 0)
	{
		Dev_NJ70_Send_Quick_Cmd(tx_buffer);
		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
		__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
	}
}

uint8_t torque_first = 0;   /* 1 = 已收到第一帧有效力矩 */
float alpha = 0.7;
float torque = 0;

/* =====================================================================
 *  重力矩补偿（2026-10-23 拟合，负载为轻质法兰）
 *
 *      tau_g(th) = a*cos(th) + b*sin(th) + c
 *
 *      a = -0.191777     标准差 +-0.034714
 *      b = -0.347195     标准差 +-0.019749
 *      c = +0.004957     标准差 +-0.036352
 *
 *      A = sqrt(a^2+b^2) = 0.3966 N.m
 *      R^2 = 0.9985    RMS 残差 7.5 mN.m
 *
 *  th 必须用 0x6064 的绝对角度（pos * 2pi / 524288），
 *  不能减任何位置偏移，拟合用的就是绝对角度。
 *  换负载必须重新拟合。
 * ===================================================================== */
#define GRAV_A          (-0.191777f)
#define GRAV_B          (-0.347195f)
#define GRAV_C          ( 0.004957f)
#define POS_PER_REV_F   (524288.0f)
#define POS_TO_RAD(p)   ((float)(p) * (6.283185307f / POS_PER_REV_F))

float Joint_GravityTorque(int32_t pos)
{
    float th = POS_TO_RAD(pos);
    return GRAV_A * cosf(th) + GRAV_B * sinf(th) + GRAV_C;
}

/* ---- 重力补偿效果观察用状态 ---- */
static uint8_t  s_gc_div  = 0;      /* rx_proc 分频计数 */
static uint8_t  s_gc_fail = 0;      /* 连续读位置失败次数 */
static uint16_t s_gc_n    = 0;      /* 统计样本数 */
static float    s_gc_sum  = 0.0f;
static float    s_gc_max  = 0.0f;
static float    s_gc_min  = 0.0f;

void rx_proc(void)
{
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
			if(torque_first == 1)
			{
				torque = Com_LowPass_Filter(alpha,torque_temp,torque);
			}
			else
			{
				torque = torque_temp;   /* 首帧直接给初值，避免从 0 爬上来 */
				torque_first = 1;
			}
		}
	}
	
	memset(rx_buffer,0,sizeof(rx_buffer));
	rx_flag = 0;
	
	/* ---- 重力补偿效果观察：每 5 帧（250 ms）读一次位置并打印 ----
	 *  res = tau - tau_g(th)
	 *   静止不碰      -> res 应该贴在 0 附近（只剩噪声和静摩擦残差）
	 *   用手拨动      -> res 跟着手的作用力矩走，符号与手一致
	 *   转到限位看    -> 角度对了 res 就小，角度算错 res 会变成好几 N.m
	 */
	if(++s_gc_div < 5)
		return;
	s_gc_div = 0;

	{
		uint32_t raw = 0;
		if(Joint_SdoReadBlock(OD_POSITION_ACTUAL, 0, &raw, 20))
		{
			float tau_g, res;
			s_gc_fail = 0;
			tau_g = Joint_GravityTorque((int32_t)raw);
			res   = torque - tau_g;

			if(s_gc_n == 0)
			{
				s_gc_max = res;
				s_gc_min = res;
			}
			if(res > s_gc_max) s_gc_max = res;
			if(res < s_gc_min) s_gc_min = res;
			s_gc_sum += res;
			s_gc_n++;

			printf("[GC] th %7.2f  tau %+7.4f  tg %+7.4f  res %+7.4f\r\n",
			       (double)((float)(int32_t)raw * 360.0f / POS_PER_REV_F),
			       (double)torque, (double)tau_g, (double)res);

			if(s_gc_n >= 40)
			{
				printf("[GC] 统计 n=%u  res均值 %+.4f  res峰峰 %.4f\r\n",
				       (unsigned)s_gc_n,
				       (double)(s_gc_sum / (float)s_gc_n),
				       (double)(s_gc_max - s_gc_min));
				s_gc_n   = 0;
				s_gc_sum = 0.0f;
			}
		}
		else
		{
			if((s_gc_fail % 20) == 0)
				printf("[GC] 读 0x6064 超时（驱动器上电了吗）\r\n");
			if(s_gc_fail < 255) s_gc_fail++;
		}
	}
}

/* =====================================================================
 *  位置单位（实测标定）
 *   输出轴一圈 = 524288 counts（19 位绝对值编码器，0x6064 直接是输出端值）
 *
 *   垂直          210466
 *   顺时针 45 度  282128   ->  +71662
 *   逆时针 45 度  151877   ->  -58589
 *   理论 45 度 = 65536；两者均值 65126（-0.6%），正反差异为机械回差（约 9 度）
 *
 *   θ[deg] = pos * 360 / 524288
 * ===================================================================== */

#define POS_PER_REV       524288.0f
#define DEG_TO_COUNT(d)   ((int32_t)((d) * POS_PER_REV / 360.0f))
#define COUNT_TO_DEG(c)   ((float)(c) * 360.0f / POS_PER_REV)

#define TEST_STEP_MS       300u     /* 状态机步进间隔 */
#define MOVE_TIMEOUT_MS   8000u     /* 单点移动超时 */

#define PROFILE_VEL      20000u     /* 0x6081 轮廓速度 counts/s = 13.7 deg/s */
#define PROFILE_ACC      50000u     /* 0x6083/0x6084 加减速度 counts/s^2 */

/* ---- 重力矩扫角采集参数 ---- */
#define SCAN_POINTS        8        /* 采集点数 */
#define SCAN_START_DEG    (-35.0f)  /* 起始角度，相对 pos0 */
#define SCAN_STEP_DEG      10.0f    /* 点间隔 */
#define SCAN_SETTLE_MS    1500u     /* 到位后稳定时间，等静差收敛 */
#define SCAN_SAMPLE_MS      50u     /* 力矩采样间隔 */

static int32_t  s_pos0     = 0;      /* 使能时的起始位置 */
static int32_t  s_target   = 0;      /* 当前目标 */
static uint32_t s_move_t   = 0;      /* 本次移动开始时刻 */
static uint8_t  s_scan_i   = 0;      /* 当前扫角点序号 */
static uint32_t s_settle_t = 0;      /* 稳定期起点 */
static float    s_sum_tau  = 0.0f;   /* 力矩累加 */
static uint16_t s_cnt_tau  = 0;      /* 力矩样本数 */
static uint32_t s_t0       = 0;      /* 首次进入 case 0 的时刻 */

/* ---------------------------------------------------------------------
 *  重力矩扫角采集（PP 模式，0x6060 = 1）
 *
 *  流程：切 PP -> 设轮廓速度/加减速 -> 目标 = 当前位置 -> 06/07/0F 使能
 *        -> 依次走到 pos0-35 到 pos0+35 共 8 个点
 *        -> 每点等 bit10 后再稳 1.5 s，期间每 50 ms 采一次力矩取平均
 *
 *  输出：DATA,idx,target_deg,act_deg,counts,torque_Nm
 *  拟合用实测角度 act_deg，不用 target_deg（位置环有约 1.5 deg 静差）
 * ------------------------------------------------------------------- */
void test_proc(void)
{
    static uint8_t  step   = 0;
    static uint32_t tick   = 0;
    uint32_t v      = 0;
    uint32_t period = 0;
    int32_t  pos    = 0;
    uint16_t sw     = 0;
    float    deg    = 0.0f;

    if (step == 99u) return;

    /* 采样阶段要快，其他阶段按 300 ms 节拍 */
    period = (step == 13u) ? SCAN_SAMPLE_MS : TEST_STEP_MS;
    if (uwTick - tick < period) return;
    tick = uwTick;

    switch (step)
    {
    case 0:     /* 等首帧力矩，再读状态字和起始位置 */
        if (s_t0 == 0u) s_t0 = uwTick;
        if (torque_first == 0u)
        {
            if ((uwTick - s_t0) > 15000u)
            {
                printf("\r\n[FAIL] 传感器 15 s 没收到有效帧，tx_proc() 打开了吗\r\n");
                step = 99u;
            }
            return;
        }

        printf("\r\n---- 重力矩扫角采集（PP 模式）----\r\n");

        if (!Joint_SdoReadBlock(OD_STATUSWORD, 0, &v, 100))
        {
            printf("[FAIL] 读状态字超时\r\n");
            step = 99u;
            return;
        }
        printf("状态字 0x%04lX\r\n", (unsigned long)v);

        if (!Joint_SdoReadBlock(OD_POSITION_ACTUAL, 0, &v, 100))
        {
            printf("[FAIL] 读位置超时\r\n");
            step = 99u;
            return;
        }
        s_pos0 = (int32_t)v;
        printf("起始位置 %ld counts  (%.2f deg)\r\n",
               (long)s_pos0, (double)COUNT_TO_DEG(s_pos0));
        printf("扫角范围 %+.1f ~ %+.1f deg，共 %u 点，每点停 %u ms\r\n",
               (double)SCAN_START_DEG,
               (double)(SCAN_START_DEG + (SCAN_POINTS - 1) * SCAN_STEP_DEG),
               (unsigned)SCAN_POINTS, (unsigned)SCAN_SETTLE_MS);
        printf("当前读数 %.4f N.m（原始值，零偏会进拟合的 c）\r\n",
               (double)torque);
        step = 1u;
        break;

    case 1:     /* 切 PP */
        if (!Joint_SdoWriteBlock(OD_MODES_OF_OPERATION, 0, JOINT_MODE_PP, 1))
        {
            printf("[FAIL] 写 0x6060=1 失败  abort=0x%08lX\r\n",
                   (unsigned long)Joint_SdoAbortCode());
            step = 99u;
            return;
        }
        step = 2u;
        break;

    case 2:     /* 确认模式 */
        if (!Joint_SdoReadBlock(OD_MODE_DISPLAY, 0, &v, 100))
        {
            printf("[FAIL] 读 0x6061 超时\r\n");
            step = 99u;
            return;
        }
        printf("当前模式 %d\r\n", (int)(int8_t)v);
        if ((int8_t)v != JOINT_MODE_PP)
        {
            printf("[FAIL] 模式没有切到 1(PP)\r\n");
            step = 99u;
            return;
        }
        step = 3u;
        break;

    case 3:     /* 轮廓速度 */
        if (!Joint_SdoWriteBlock(OD_PROFILE_VELOCITY, 0, PROFILE_VEL, 4))
        {
            printf("[FAIL] 写 0x6081 失败\r\n");
            step = 99u;
            return;
        }
        printf("轮廓速度 %u counts/s (%.1f deg/s)\r\n",
               (unsigned int)PROFILE_VEL, (double)COUNT_TO_DEG((int32_t)PROFILE_VEL));
        step = 4u;
        break;

    case 4:     /* 加速度 */
        if (!Joint_SdoWriteBlock(OD_PROFILE_ACCEL, 0, PROFILE_ACC, 4))
        {
            printf("[FAIL] 写 0x6083 失败\r\n");
            step = 99u;
            return;
        }
        step = 5u;
        break;

    case 5:     /* 减速度 */
        if (!Joint_SdoWriteBlock(OD_PROFILE_DECEL, 0, PROFILE_ACC, 4))
        {
            printf("[FAIL] 写 0x6084 失败\r\n");
            step = 99u;
            return;
        }
        printf("加减速度 %u counts/s^2\r\n", (unsigned int)PROFILE_ACC);
        step = 6u;
        break;

    case 6:     /* 目标 = 当前位置，避免使能瞬间跳 */
        s_target = s_pos0;
        if (!Joint_SdoWriteBlock(OD_TARGET_POSITION, 0, (uint32_t)s_target, 4))
        {
            printf("[FAIL] 写 0x607A 失败\r\n");
            step = 99u;
            return;
        }
        step = 7u;
        break;

    case 7:     /* Shutdown */
        Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_SHUTDOWN, 2);
        step = 8u;
        break;

    case 8:     /* Switch On */
        Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_SWITCH_ON, 2);
        step = 9u;
        break;

    case 9:     /* Enable Operation */
        Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_ENABLE_OPERATION, 2);
        step = 10u;
        break;

    case 10:    /* 确认使能 */
        if (!Joint_SdoReadBlock(OD_STATUSWORD, 0, &v, 100))
        {
            printf("[FAIL] 读状态字超时\r\n");
            step = 99u;
            return;
        }
        printf("状态字 0x%04lX  状态位 0x%02lX\r\n",
               (unsigned long)v, (unsigned long)(v & SW_MASK_STATE));

        if ((v & SW_MASK_STATE) != SW_STATE_ENABLED)
        {
            printf("[FAIL] 没进 Operation enabled(0x27)\r\n");
            printf("       查 STO_IN(PIN1)/STO_COM(PIN2) 是否短接、48V 是否到位\r\n");
            step = 99u;
            return;
        }

        printf("[ OK ] 已使能，开始扫角\r\n");
        printf("DATA,idx,target_deg,act_deg,counts,tau_Nm\r\n");
        s_scan_i = 0u;
        step = 11u;
        break;

    case 11:    /* 发第 s_scan_i 个点 */
        if (s_scan_i >= SCAN_POINTS)
        {
            step = 14u;
            break;
        }
        deg = SCAN_START_DEG + (float)s_scan_i * SCAN_STEP_DEG;
        s_target = s_pos0 + DEG_TO_COUNT(deg);

        Joint_SdoWriteBlock(OD_TARGET_POSITION, 0, (uint32_t)s_target, 4);
        Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_ENABLE_OPERATION, 2);                    /* bit4 = 0 */
        Joint_SdoWriteBlock(OD_CONTROLWORD, 0, CW_ENABLE_OPERATION | CW_NEW_SETPOINT, 2);  /* bit4 上升沿 */

        printf("--> 第 %u 点  目标 %+6.1f deg  (%+8ld)\r\n",
               (unsigned)s_scan_i, (double)deg, (long)s_target);
        s_move_t = uwTick;
        step = 12u;
        break;

    case 12:    /* 等到位：先等 600ms 让 bit10 确实清 0 */
        if ((uwTick - s_move_t) < 600u) break;

        if (!Joint_SdoReadBlock(OD_STATUSWORD, 0, &v, 100))
        {
            printf("[WARN] 读状态字超时，跳过第 %u 点\r\n", (unsigned)s_scan_i);
            s_scan_i++;
            step = 11u;
            break;
        }
        sw = (uint16_t)v;

        if ((sw & 0x0400u) != 0u)           /* bit10 target reached */
        {
            s_settle_t = uwTick;
            s_sum_tau  = 0.0f;
            s_cnt_tau  = 0u;
            step = 13u;
            break;
        }
        if ((uwTick - s_move_t) > MOVE_TIMEOUT_MS)
        {
            printf("[WARN] 第 %u 点超时未到位，跳过\r\n", (unsigned)s_scan_i);
            s_scan_i++;
            step = 11u;
        }
        break;

    case 13:    /* 稳定期采样，50ms 一次 */
        s_sum_tau += torque;
        s_cnt_tau++;

        if ((uwTick - s_settle_t) < SCAN_SETTLE_MS) break;

        if (!Joint_SdoReadBlock(OD_POSITION_ACTUAL, 0, &v, 100))
        {
            printf("[WARN] 第 %u 点读位置超时\r\n", (unsigned)s_scan_i);
            s_scan_i++;
            step = 11u;
            break;
        }
        pos = (int32_t)v;
        printf("DATA,%u,%.2f,%.2f,%ld,%.4f\r\n",
               (unsigned)s_scan_i,
               (double)(SCAN_START_DEG + (float)s_scan_i * SCAN_STEP_DEG),
               (double)COUNT_TO_DEG(pos),
               (long)pos,
               s_cnt_tau ? (double)(s_sum_tau / (float)s_cnt_tau) : 0.0);

        s_scan_i++;
        step = 11u;
        break;

    case 14:    /* 汇总 */
        printf("---- 采集结束，共 %u 点 ----\r\n", (unsigned)SCAN_POINTS);
        printf("把 DATA 行贴出来，拟合 tau = a*cos(theta) + b*sin(theta) + c\r\n");
        printf("其中 c = 传感器零偏 + 装夹预紧，与角度无关\r\n");
        step = 99u;
        break;

    default:
        step = 99u;
        break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
		rx_flag = 1;
    }
}

/* ---------------- CAN 接收回调 ----------------
 * 中断上下文：只取帧 + 分派，不要在这里 printf
 * 协议解析全部在 joint.c 里做
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader = {0};
    uint8_t rxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;
    if (rxHeader.IDE == CAN_ID_STD)
        Joint_RxDispatch((uint16_t)rxHeader.StdId, rxData, (uint8_t)rxHeader.DLC);
}




