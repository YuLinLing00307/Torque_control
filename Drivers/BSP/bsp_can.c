#include "bsp_can.h"

void Bsp_Can_Init(void)
{
	/* 1. 配置筛选器 */
    CAN_FilterTypeDef sFilterConfig = {0};                              // 过滤器配置结构体
    sFilterConfig.FilterBank = 0;                                       // 筛选器组编号
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                   // 掩码模式
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                  // 32位模式
    sFilterConfig.FilterIdHigh         =  0x0000;                       // ID(验证码)的高16位
    sFilterConfig.FilterIdLow          =  0x0000;                       // ID(验证码)的低16位
    sFilterConfig.FilterMaskIdHigh     =  0x0000;                       // 掩码高16位; 位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterMaskIdLow      =  0x0000;                       // 掩码低16位；位匹配，位0-此位都通过、位1-此位需要与ID位相同
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                  // 分配到 FIFO 0
    sFilterConfig.FilterActivation = ENABLE;                            // 使能筛选器
    sFilterConfig.SlaveStartFilterBank = 14;                            // 设置CAN2的筛选器起始编号; CAN1、CAN2共用28个筛选器(0~27)，默认情况下：CAN1用0~13、CAN2必须14~27。此值的设置范围14~27, 即必须14起步。因为CAN1、CAN2共用28个筛选器(0~27)，设置后，CAN1可用筛选器为(0~值-1), CAN2可用筛选器为(值~27); 标准库没有这个选项，CAN1默认0~13, CAN2默认14~27; 当使用寄存器操作时最自由，CAN1可以0~27, CAN2必须14~27。
 
    /* 2. 初始化筛选器 */
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        printf("CAN1筛选器初始化失败！\r\n");
        // Error_Handler();
    }
 
    /* 3. 使能FIFO0接收中断 */
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("CAN1接收中断使能失败！\r\n");
        // Error_Handler();
    }
	
	HAL_CAN_Start(&hcan);
}


uint8_t Bsp_Can_SendData(uint32_t type, uint32_t ID,uint8_t *msgData,uint8_t num)
{
    // 定义两个变量
    static CAN_TxHeaderTypeDef canTxHeader = {0};             // CAN发送消息的结构体，定义了发送报文的关键参数
    static uint32_t txMailbox = 0;                            // 用于记录发送成功时所用的邮箱编号：0~2; 被发送函数HAL_CAN_AddTxMessage()赋值
                                                              
    /* 限制数据的字节数 */                                    
    if (num > 8)                                              // 判断字节是否超过8字节
        num = 8;                                              // 如果超过8字节，只发送前8个字节
                                                              
    /* 1.配置帧信息 */                                        
    if (type == CAN_ID_EXT)                                   // 扩展帧
    {                                                         
        canTxHeader.ExtId = ID ;                              // 扩展帧ID; 值有效范围：0x00000000~0x1FFFFFFF
        canTxHeader.IDE   = CAN_ID_EXT;                       // 帧格式：扩展帧(CAN_ID_EXT); 此值是HAL库文件stm32f4xx_hal.h里的宏定义
    }                                                         
    else                                                      // 标准帧
    {                                                         
        canTxHeader.StdId = ID;                               // 帧ID; 值有效范围：0x000~0x7FF
        canTxHeader.IDE   = CAN_ID_STD;                       // 帧格式: 标准帧(CAN_ID_STD); 此值是HAL库文件stm32f4xx_hal.h里的宏定义
    }                                                         
    canTxHeader.RTR   = CAN_RTR_DATA;                         // 数据帧
    canTxHeader.DLC   = num;                                  // 数据字节数
    canTxHeader.TransmitGlobalTime = DISABLE;                 // 使能时间戳添加到最后两个字节：Data[6]、Data[7]
                                                              
    /* 2.等待发送邮箱空闲 */    
    uint8_t times = 0;                                        // 注意：如果发送超时失败，总线一直在忙的可能性极低，更大的可能是，CAN没有连接到收发器                              
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)      // 每次发送前，要等待至有发送邮箱空闲。共有3个发送邮箱。如果返回值为0，即没有发送邮箱空闲，继续等
    {                                                         
        HAL_Delay(1);
        if (times++ >= 5)
        {
            times = 0;
            printf("\r发送失败：等待超时！检查CAN1的线路是否已连接到收发器\r");
            return 3;
        }
    }
 
    return HAL_CAN_AddTxMessage(&hcan, &canTxHeader, msgData, &txMailbox);     // 返回发送状态; 成功-0、错误-1、忙错误-2、超时-3
}
