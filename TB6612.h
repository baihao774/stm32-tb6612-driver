/****************************************************************************
 * @file    TB6612.h
 * @brief   STM32F103 + TB6612FNG 直流电机驱动示例（头文件）
 * @details 基于标准外设库的 TB6612FNG 直流电机驱动，使用 A 通道，
 *          支持 PWM 调速、正反转、刹车/滑行停止、待机控制与
 *          软启动斜坡，可作为直流电机驱动类设计的参考示例代码。
 * @author  baihao774
 * @license MIT
 * @note    完整说明见 GitHub 仓库 README
 ****************************************************************************/

#ifndef __TB6612_H
#define __TB6612_H

#include "stm32f10x.h"

/*---------------- 硬件连接配置（可修改） ----------------
   驱动芯片：TB6612FNG（双 H 桥电机驱动，本示例使用 A 通道）
   接线：
     PWMA -> PB0（TIM3_CH3 默认引脚，无需重映射，PWM 调速输入）
     AIN1 -> PB1（方向控制 1，推挽输出）
     AIN2 -> PB5（方向控制 2，推挽输出）
     STBY -> PB7（待机控制，高电平使能驱动、低电平待机）
     VM   -> 电机电源正（按电机额定电压单独供电，切勿取自单片机 3.3V）
     VCC  -> 3.3V（逻辑电源）
     AO1/AO2 -> 电机两端
     模块 GND 必须与单片机 GND 共地！
   TB6612 逻辑真值表（AIN1/AIN2/PWMA -> 输出）：
     1/0/PWM -> 正转调速；0/1/PWM -> 反转调速；
     1/1/x   -> 短接刹车；0/0/x   -> 滑行停止；STBY=0 -> 待机（输出高阻）
*/
/* PWMA：PWM 调速输入（PB0 / TIM3_CH3） */
#define TB6612_GPIO_PORT		GPIOB
#define TB6612_GPIO_PIN			GPIO_Pin_0
#define TB6612_GPIO_CLK			RCC_APB2Periph_GPIOB
#define TB6612_TIM				TIM3
#define TB6612_TIM_CLK			RCC_APB1Periph_TIM3

/* AIN1 / AIN2：方向控制脚；STBY：待机使能脚（均为普通推挽输出） */
#define TB6612_DIR_GPIO_PORT	GPIOB
#define TB6612_DIR_GPIO_CLK		RCC_APB2Periph_GPIOB
#define TB6612_AIN1_PIN			GPIO_Pin_1
#define TB6612_AIN2_PIN			GPIO_Pin_5
#define TB6612_STBY_PIN			GPIO_Pin_7

/* TB6612 控制电平操作宏 */
#define TB6612_AIN1_H()			GPIO_SetBits(TB6612_DIR_GPIO_PORT, TB6612_AIN1_PIN)
#define TB6612_AIN1_L()			GPIO_ResetBits(TB6612_DIR_GPIO_PORT, TB6612_AIN1_PIN)
#define TB6612_AIN2_H()			GPIO_SetBits(TB6612_DIR_GPIO_PORT, TB6612_AIN2_PIN)
#define TB6612_AIN2_L()			GPIO_ResetBits(TB6612_DIR_GPIO_PORT, TB6612_AIN2_PIN)
#define TB6612_STBY_H()			GPIO_SetBits(TB6612_DIR_GPIO_PORT, TB6612_STBY_PIN)
#define TB6612_STBY_L()			GPIO_ResetBits(TB6612_DIR_GPIO_PORT, TB6612_STBY_PIN)

/* TIM3 挂载在 APB1 总线上：本示例 APB1 = 36MHz，定时器时钟为其 2 倍即 72MHz。
   不分频计数，72MHz / (3599 + 1) = 20kHz，占空比分辨率为 1/3600，人耳听不到啸叫。
   TB6612FNG 的 PWM 输入频率最高支持 100kHz，20kHz 完全满足要求。 */
#define TB6612_TIM_PRESCALER	0			/* 预分频值：不分频 */
#define TB6612_TIM_PERIOD		3599		/* 自动重装值 */

/* 占空比（0~100）转换为比较值（0 ~ TB6612_TIM_PERIOD+1） */
#define TB6612_DUTY_TO_CCR(duty)	((uint16_t)((uint32_t)(duty) * (TB6612_TIM_PERIOD + 1) / 100))

/*---------------- 软启动配置（可修改） ----------------
   升速时每个更新周期把占空比增加 TB6612_SOFT_START_STEP 个百分点，
   用于抑制电机启动瞬间的冲击电流，避免共用电源被拉低导致单片机复位。
   经验值：电机单独供电时可设 8~10；共用电源建议 3~5；
   设为 0 表示关闭软启动（直接跳到目标占空比）。 */
#define TB6612_SOFT_START_STEP	5

/*---------------- 电机运行方向 ----------------*/
#define TB6612_DIR_FORWARD		0		/* 正转（AIN1=1，AIN2=0） */
#define TB6612_DIR_REVERSE		1		/* 反转（AIN1=0，AIN2=1） */

/*---------------- 接口函数 ----------------*/

/**
  * @brief  初始化 TB6612 驱动（PWM 输出 TIM3_CH3 -> PB0，方向脚 PB1/PB5，待机脚 PB7）
  * @param  无
  * @retval 无
  * @note   初始化后 TB6612 处于待机状态（STBY=0，输出高阻），电机不转
  */
void TB6612_Init(void);

/**
  * @brief  设置电机运行方向
  * @param  dir: TB6612_DIR_FORWARD 正转 / TB6612_DIR_REVERSE 反转
  * @retval 无
  * @note   建议在停转或低速状态下切换方向，避免换向冲击电流
  */
void TB6612_SetDir(uint8_t dir);

/**
  * @brief  设置电机 PWM 占空比（带软启动：升速走斜坡，降速立即生效）
  * @param  duty_percent: 占空比，0~100（%），大于 100 按 100 处理
  * @retval 无
  * @note   占空比为 0 时电机滑行停止；升速斜坡由 TB6612_Update 推进
  */
void TB6612_SetDuty(uint8_t duty_percent);

/**
  * @brief  读取当前电机占空比
  * @param  无
  * @retval 当前占空比，0~100（%）
  */
uint8_t TB6612_GetDuty(void);

/**
  * @brief  软启动斜坡更新（需在主循环中以固定周期调用，如每 20ms 一次）
  * @param  无
  * @retval 无
  * @note   升速过程中每个调用周期把占空比增加 TB6612_SOFT_START_STEP 个百分点，
  *         直到达到目标占空比；降速/停转立即生效，不做斜坡。
  */
void TB6612_Update(void);

/**
  * @brief  短接刹车（AIN1=1，AIN2=1，电机快速停转）
  * @param  无
  * @retval 无
  */
void TB6612_Brake(void);

/**
  * @brief  滑行停止（占空比清零，电机靠惯性滑行停止）
  * @param  无
  * @retval 无
  */
void TB6612_Coast(void);

/**
  * @brief  待机控制
  * @param  enable: 1 = 使能驱动（STBY=1）；0 = 待机（STBY=0，输出高阻，零功耗）
  * @retval 无
  */
void TB6612_Standby(uint8_t enable);

#endif
