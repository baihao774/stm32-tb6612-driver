/****************************************************************************
 * @file    TB6612.c
 * @brief   STM32F103 + TB6612FNG 直流电机驱动示例（源文件）
 * @details 功能：
 *            - TIM3_CH3 输出 20kHz PWM 调速；
 *            - 正转 / 反转方向控制；
 *            - 短接刹车、滑行停止、待机三种停止方式；
 *            - 升速软启动斜坡，抑制电机启动冲击电流。
 * @author  baihao774
 * @license MIT
 ****************************************************************************/

#include "TB6612.h"

/*---------------- 模块内部状态 ----------------*/
static uint8_t g_tb6612_dir          = TB6612_DIR_FORWARD;	/* 当前运行方向 */
static uint8_t g_tb6612_duty         = 0;					/* 当前实际占空比 */
static uint8_t g_tb6612_target_duty  = 0;					/* 软启动目标占空比 */
static uint8_t g_tb6612_ramp_active  = 0;					/* 软启动斜坡进行标志 */

/**
  * @brief  立即把占空比写入比较寄存器并更新 TB6612 控制脚（内部函数，不做软启动）
  * @param  duty_percent: 占空比，0~100（%），大于 100 按 100 处理
  * @retval 无
  * @note   占空比为 0 时 TB6612 进入待机（STBY=0，输出高阻），电机滑行停止且零功耗；
  *         占空比大于 0 时 STBY=1，按当前方向输出 PWM 调速
  */
static void TB6612_ApplyDuty(uint8_t duty_percent)
{
	if (duty_percent > 100)
		duty_percent = 100;

	if (duty_percent == g_tb6612_duty)	/* 占空比未变化则不重复写寄存器 */
		return;

	g_tb6612_duty = duty_percent;

	if (duty_percent == 0)
	{
		/* 停转：先关 PWM，再拉低 STBY 使 TB6612 输出高阻，避免低速爬行和静态功耗 */
		TIM_SetCompare3(TB6612_TIM, 0);
		TB6612_AIN1_L();
		TB6612_AIN2_L();
		TB6612_STBY_L();
	}
	else
	{
		/* 运行：TB6612 使能，按当前方向设置 AIN1/AIN2，PWM 输出到 PWMA */
		if (g_tb6612_dir == TB6612_DIR_FORWARD)
		{
			TB6612_AIN1_H();
			TB6612_AIN2_L();
		}
		else
		{
			TB6612_AIN1_L();
			TB6612_AIN2_H();
		}
		TB6612_STBY_H();
		/* 100% 时比较值大于重装值（输出恒高、全速） */
		TIM_SetCompare3(TB6612_TIM, TB6612_DUTY_TO_CCR(duty_percent));
	}
}

/**
  * @brief  初始化 TB6612 驱动（PWM 输出 TIM3_CH3 -> PB0，方向脚 PB1/PB5，待机脚 PB7）
  * @param  无
  * @retval 无
  * @note   初始化后 TB6612 处于待机状态（STBY=0，输出高阻），电机不转
  */
void TB6612_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(TB6612_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(TB6612_DIR_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(TB6612_TIM_CLK, ENABLE);

	/* PB0 配置为复用推挽输出，作为 TIM3_CH3 的 PWM 输出脚（接 TB6612 的 PWMA） */
	GPIO_InitStructure.GPIO_Pin   = TB6612_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TB6612_GPIO_PORT, &GPIO_InitStructure);

	/* AIN1 / AIN2 / STBY 配置为普通推挽输出，先全部拉低：TB6612 待机，输出高阻，电机不转 */
	GPIO_InitStructure.GPIO_Pin   = TB6612_AIN1_PIN | TB6612_AIN2_PIN | TB6612_STBY_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TB6612_DIR_GPIO_PORT, &GPIO_InitStructure);
	TB6612_AIN1_L();
	TB6612_AIN2_L();
	TB6612_STBY_L();

	/* 时基单元：计数时钟 72MHz，重装值 3599，PWM 频率 = 72MHz / 3600 = 20kHz */
	TIM_TimeBaseStructure.TIM_Prescaler     = TB6612_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period        = TB6612_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TB6612_TIM, &TIM_TimeBaseStructure);

	/* 通道 3 输出 PWM1 波形，初始比较值为 0（占空比 0%，电机不转） */
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse       = 0;
	TIM_OC3Init(TB6612_TIM, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TB6612_TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TB6612_TIM, ENABLE);
	TIM_Cmd(TB6612_TIM, ENABLE);

	/* 上电默认：正转方向、占空比 0 停转 */
	g_tb6612_dir         = TB6612_DIR_FORWARD;
	g_tb6612_duty        = 0;
	g_tb6612_target_duty = 0;
	g_tb6612_ramp_active = 0;
}

/**
  * @brief  设置电机运行方向
  * @param  dir: TB6612_DIR_FORWARD 正转 / TB6612_DIR_REVERSE 反转
  * @retval 无
  */
void TB6612_SetDir(uint8_t dir)
{
	if (dir > TB6612_DIR_REVERSE)
		dir = TB6612_DIR_FORWARD;

	if (dir == g_tb6612_dir)
		return;

	g_tb6612_dir = dir;

	/* 电机正在运行时换向：立即按新方向重写控制脚 */
	if (g_tb6612_duty > 0)
	{
		uint8_t duty = g_tb6612_duty;

		g_tb6612_duty = 0;				/* 强制 ApplyDuty 重新写寄存器 */
		TB6612_ApplyDuty(duty);
	}
}

/**
  * @brief  设置电机 PWM 占空比（带软启动：升速走斜坡，降速立即生效）
  * @param  duty_percent: 占空比，0~100（%），大于 100 按 100 处理
  * @retval 无
  */
void TB6612_SetDuty(uint8_t duty_percent)
{
	if (duty_percent > 100)
		duty_percent = 100;

	g_tb6612_target_duty = duty_percent;

#if (TB6612_SOFT_START_STEP > 0)
	if (duty_percent > g_tb6612_duty)
	{
		g_tb6612_ramp_active = 1;		/* 升速：由 TB6612_Update 逐步抬升，减小启动冲击电流 */
		return;
	}
#endif

	/* 降速、停转或未启用软启动：立即输出目标占空比 */
	g_tb6612_ramp_active = 0;
	TB6612_ApplyDuty(duty_percent);
}

/**
  * @brief  读取当前电机占空比
  * @param  无
  * @retval 当前占空比，0~100（%）
  */
uint8_t TB6612_GetDuty(void)
{
	return g_tb6612_duty;
}

/**
  * @brief  软启动斜坡更新（需在主循环中以固定周期调用，如每 20ms 一次）
  * @param  无
  * @retval 无
  */
void TB6612_Update(void)
{
	uint8_t duty;

	if (!g_tb6612_ramp_active)			/* 未在软启动过程中，无需处理 */
		return;

	duty = g_tb6612_duty + TB6612_SOFT_START_STEP;
	if (duty >= g_tb6612_target_duty)
	{
		duty = g_tb6612_target_duty;
		g_tb6612_ramp_active = 0;		/* 到达目标占空比，软启动结束 */
	}

	TB6612_ApplyDuty(duty);
}

/**
  * @brief  短接刹车（AIN1=1，AIN2=1，电机快速停转）
  * @param  无
  * @retval 无
  */
void TB6612_Brake(void)
{
	g_tb6612_target_duty = 0;
	g_tb6612_ramp_active = 0;
	g_tb6612_duty        = 0;

	TIM_SetCompare3(TB6612_TIM, 0);
	TB6612_STBY_H();					/* 刹车需要驱动使能 */
	TB6612_AIN1_H();
	TB6612_AIN2_H();
}

/**
  * @brief  滑行停止（占空比清零，电机靠惯性滑行停止）
  * @param  无
  * @retval 无
  */
void TB6612_Coast(void)
{
	g_tb6612_target_duty = 0;
	g_tb6612_ramp_active = 0;
	TB6612_ApplyDuty(0);
}

/**
  * @brief  待机控制
  * @param  enable: 1 = 使能驱动（STBY=1）；0 = 待机（STBY=0，输出高阻，零功耗）
  * @retval 无
  * @note   待机后再次使能时，按内部保存的占空比恢复输出
  */
void TB6612_Standby(uint8_t enable)
{
	if (enable)
	{
		uint8_t duty = g_tb6612_duty;

		if (duty > 0)
		{
			g_tb6612_duty = 0;			/* 强制 ApplyDuty 重新写寄存器 */
			TB6612_ApplyDuty(duty);
		}
		else
		{
			TB6612_STBY_H();
		}
	}
	else
	{
		TIM_SetCompare3(TB6612_TIM, 0);
		TB6612_AIN1_L();
		TB6612_AIN2_L();
		TB6612_STBY_L();
	}
}
