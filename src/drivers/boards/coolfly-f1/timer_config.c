/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file px4fmu_timer_config.c
 *
 * Configuration data for the stm32 pwm_servo, input capture and pwm input driver.
 *
 * Note that these arrays must always be fully-sized.
 */

#include <stdint.h>

#include <chip.h>
#include <ar_gpio.h>
#include <ar_tim.h>
#include <chip/ar_tim.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/artosyn/drv_io_timer.h>

#include "board_config.h"


#define ar_timer_freq   128000000


/** TIMER 0 当做了 alarm **/
__EXPORT const io_timers_t io_timers[MAX_IO_TIMERS] = {
	// {
	// 	.base = AR_TIM0_BASE,
	// 	.clock_freq = ar_timer_freq,
	// 	.handler = io_timer_handler,
	// 	.vectorno =  AR_IRQ_TIM0_CH0,
	// 	.gpio_out = GP_TIM0_CH0OUT,
	// 	.gpio_in  = GP_TIM0_CH0IN,
	// 	.channel_index = 0,
	// 	.channel_mode = IOTimerChanMode_NotUsed,

	// 	.rTIM_LOADCOUNT = AR_TIM0_TIMERLOADCOUNT_OFFSET,
	// 	.rTIM_LOADCOUNT2 = AR_TIM0_TIMERLOADCOUNT2_OFFSET,
	// 	.rTIM_CURRENTVALUE = AR_TIM0_TIMERCURRENTVALUE_OFFSET,
	// 	.rTIM_CONTROLREG = AR_TIM0_TIMERCONTROLREG_OFFSET,
	// 	.rTIM_EOI = AR_TIM0_TIMEREOI_OFFSET,
	// 	.rTIM_INTSTATUS = AR_TIM0_TIMERINTSTATUS_OFFSET
	// },
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM0_CH1,
		.gpio_out = GP_TIM0_CH1OUT,
		.gpio_in  = GP_TIM0_CH1IN,
		.channel_index = 0,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM1_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM1_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM1_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM1_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM1_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM1_TIMERINTSTATUS_OFFSET
	},
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM0_CH2,
		.gpio_out = GP_TIM0_CH2OUT,
		.gpio_in  = GP_TIM0_CH2IN,
		.channel_index = 1,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM2_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM2_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM2_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM2_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM2_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM2_TIMERINTSTATUS_OFFSET
	},
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM0_CH3,
		.gpio_out = GP_TIM0_CH3OUT,
		.gpio_in  = GP_TIM0_CH3IN,
		.channel_index = 2,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM3_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM3_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM3_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM3_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM3_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM3_TIMERINTSTATUS_OFFSET

	},
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno = AR_IRQ_TIM0_CH4,
		.gpio_out = GP_TIM0_CH4OUT,
		.gpio_in  = GP_TIM0_CH4IN,
		.channel_index = 3,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM4_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM4_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM4_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM4_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM4_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM4_TIMERINTSTATUS_OFFSET

	},
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM0_CH5,
		.gpio_out = GP_TIM0_CH5OUT,
		.gpio_in  = GP_TIM0_CH5IN,
		.channel_index = 4,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM5_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM5_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM5_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM5_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM5_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM5_TIMERINTSTATUS_OFFSET
	},
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM0_CH6,
		.gpio_out = GP_TIM0_CH6OUT,
		.gpio_in  = GP_TIM0_CH6IN,
		.channel_index = 5,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM6_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM6_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM6_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM6_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM6_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM6_TIMERINTSTATUS_OFFSET
	},
	{
		.base = AR_TIM0_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM0_CH7,
		.gpio_out = GP_TIM0_CH7OUT,
		.gpio_in  = GP_TIM0_CH7IN,
		.channel_index = 6,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM7_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM7_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM7_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM7_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM7_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM7_TIMERINTSTATUS_OFFSET
	},
	{
		.base = AR_TIM1_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM1_CH0,
		.gpio_out = GP_TIM1_CH0OUT,
		.gpio_in  = GP_TIM1_CH0IN,
		.channel_index = 7,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM10_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM10_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM10_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM10_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM10_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM10_TIMERINTSTATUS_OFFSET
	},
	{
		.base = AR_TIM1_BASE,
		.clock_freq = ar_timer_freq,
		.handler = io_timer_handler,
		.vectorno =  AR_IRQ_TIM1_CH1,
		.gpio_out = GP_TIM1_CH1OUT,
		.gpio_in  = GP_TIM1_CH1IN,
		.channel_index = 8,
		.channel_mode = IOTimerChanMode_NotUsed,

		.rTIM_LOADCOUNT = AR_TIM11_TIMERLOADCOUNT_OFFSET,
		.rTIM_LOADCOUNT2 = AR_TIM11_TIMERLOADCOUNT2_OFFSET,
		.rTIM_CURRENTVALUE = AR_TIM11_TIMERCURRENTVALUE_OFFSET,
		.rTIM_CONTROLREG = AR_TIM11_TIMERCONTROLREG_OFFSET,
		.rTIM_EOI = AR_TIM10_TIMEREOI_OFFSET,
		.rTIM_INTSTATUS = AR_TIM11_TIMERINTSTATUS_OFFSET
	}
};


