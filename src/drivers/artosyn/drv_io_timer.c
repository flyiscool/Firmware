/****************************************************************************
 *
 *   Copyright (C) 2012, 2017 PX4 Development Team. All rights reserved.
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
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include "drv_io_timer.h"

#include <ar_gpio.h>
#include <ar_tim.h>
#include <chip/ar_tim.h>
#include <chip/ar_config.h>

#define arraySize(a) (sizeof((a)) / sizeof(((a)[0])))

/* If the timer clock source provided as clock_freq is the STM32_APBx_TIMx_CLKIN
 * then configure the timer to free-run at 1MHz.
 * Otherwise, other frequencies are attainable by adjusting .clock_freq accordingly.
 * For instance .clock_freq = 1000000 would set the prescaler to 1.
 * We also allow for overrides here but all timer register usage need to be
 * taken into account
 */
#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

#define MAX_CHANNELS_PER_TIMER 1

#define _REG32(_base, _reg) (*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg) _REG32(io_timers[_tmr].base, _reg)

// #define CHANNER_IN_TIMER_OFFSET(channel) ? (channel < 8) ? channel : (channel - 8)
// #define TIMER_BASE(channel) ((channel<8) ? AR_TIM0_BASE : AR_TIM1_BASE)

// #define TIMER_LOAD_COUNT(channel) (CHANNER_IN_TIMER_OFFSET(channel))*0x14 + 0x00 + TIMER_BASE(channel)
// #define TIMER_LOAD_COUNT2(channel) (CHANNER_IN_TIMER_OFFSET(channel))*0x04 + 0xB0 + TIMER_BASE(channel)
// #define TIMER_CURRENT_VALUE(channel) (CHANNER_IN_TIMER_OFFSET(channel))*0x14 + 0x04 + TIMER_BASE(channel)
// #define TIMER_CONTROL_REG(channel) (CHANNER_IN_TIMER_OFFSET(channel))*0x14 + 0x08 + TIMER_BASE(channel)
// #define TIMER_EOI(channel) (CHANNER_IN_TIMER_OFFSET(channel))*0x014 + 0x0C + TIMER_BASE(channel)
// #define TIMER_INT_STATUS(channel) (CHANNER_IN_TIMER_OFFSET(channel))*0x014 + 0x10 + TIMER_BASE(channel)

typedef enum io_timer_reg_t
{
	rTIM_LOADCOUNT,
	rTIM_LOADCOUNT2,
	rTIM_CURRENTVALUE,
	rTIM_CONTROLREG,
	rTIM_EOI,
	rTIM_INTSTATUS
} io_timer_reg_t;

static void set_timer_reg(unsigned timer, io_timer_reg_t reg, uint32_t value)
{
	uint32_t reg_addr = 0;

	switch (reg)
	{
	case rTIM_LOADCOUNT:
		reg_addr = io_timers[timer].rTIM_LOADCOUNT;
		break;
	case rTIM_LOADCOUNT2:
		reg_addr = io_timers[timer].rTIM_LOADCOUNT2;
		break;
	case rTIM_CURRENTVALUE:
		reg_addr = io_timers[timer].rTIM_CURRENTVALUE;
		break;
	case rTIM_CONTROLREG:
		reg_addr = io_timers[timer].rTIM_CONTROLREG;
		break;
	case rTIM_EOI:
		reg_addr = io_timers[timer].rTIM_EOI;
		break;
	case rTIM_INTSTATUS:
		reg_addr = io_timers[timer].rTIM_INTSTATUS;
		break;
	default:
		break; //error
	}

	putreg32(value, reg_addr);
}

static uint32_t get_timer_reg(unsigned timer, io_timer_reg_t reg)
{	
	uint32_t reg_addr = 0;

	switch (reg)
	{
	case rTIM_LOADCOUNT:
		reg_addr = io_timers[timer].rTIM_LOADCOUNT;
		break;
	case rTIM_LOADCOUNT2:
		reg_addr = io_timers[timer].rTIM_LOADCOUNT2;
		break;
	case rTIM_CURRENTVALUE:
		reg_addr = io_timers[timer].rTIM_CURRENTVALUE;
		break;
	case rTIM_CONTROLREG:
		reg_addr = io_timers[timer].rTIM_CONTROLREG;
		break;
	case rTIM_EOI:
		reg_addr = io_timers[timer].rTIM_EOI;
		break;
	case rTIM_INTSTATUS:
		reg_addr = io_timers[timer].rTIM_INTSTATUS;
		break;
	default:
		break; //error
	}
	return getreg32(reg_addr);
}

//												 				  NotUsed   PWMOut   OneShot Trigger
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = {MAX_CHANNELS_BITS, 0, 0, 0};

typedef uint8_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_allocation_t once = 0;

int io_timer_handler(int irq, void *context, void *arg)
{
	io_timers_t *io_timer = (io_timers_t *)arg;
	get_timer_reg(io_timer->channel_index, rTIM_EOI);

	// get_timer_reg(0, rTIM_EOI);
	// get_timer_reg(1, rTIM_EOI);
	// get_timer_reg(2, rTIM_EOI);
	// get_timer_reg(3, rTIM_EOI);
	// get_timer_reg(4, rTIM_EOI);
	// get_timer_reg(5, rTIM_EOI);
	// get_timer_reg(6, rTIM_EOI);
	// get_timer_reg(7, rTIM_EOI);
	// get_timer_reg(8, rTIM_EOI);
	// get_timer_reg(9, rTIM_EOI);

	return 0;
}

static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

static inline int is_timer_uninitalized(unsigned timer)
{
	int rv = 0;

	if (once & 1 << timer)
	{
		rv = -EBUSY;
	}

	return rv;
}

static inline void set_timer_initalized(unsigned timer)
{
	once |= 1 << timer;
}

static inline void set_timer_deinitalized(unsigned timer)
{
	once &= ~(1 << timer);
}

static inline int channels_timer(unsigned channel)
{
	return channel;
}

static uint32_t get_timer_channels(unsigned timer)
{
	uint32_t channels = 0;
	static uint32_t channels_cache[MAX_IO_TIMERS] = {0};

	if (validate_timer_index(timer) < 0)
	{
		return channels;
	}
	else
	{
		if (channels_cache[timer] == 0)
		{
			const io_timers_t *tmr = &io_timers[timer];

			channels_cache[timer] = 1 << (tmr->channel_index);
		}
	}

	return channels_cache[timer];
}

static inline int is_channels_timer_uninitalized(unsigned channel)
{
	return is_timer_uninitalized(channels_timer(channel));
}

int io_timer_is_channel_free(unsigned channel)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0)
	{
		io_timers_t *io_timer = (io_timers_t *)(&io_timers[channel]);

		if (io_timer->channel_mode != IOTimerChanMode_NotUsed)
		{
			rv = -EBUSY;
		}
	}

	return rv;
}

int io_timer_validate_channel_index(unsigned channel)
{
	return channel >= MAX_IO_TIMERS;
}

int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	io_timer_channel_allocation_t channels = 0;

	io_timers_t *io_timer;

	for (uint8_t i = 0; i < MAX_IO_TIMERS; i++)
	{
		io_timer = (io_timers_t *)(&io_timers[i]);
		if (io_timer->channel_mode == mode)
		{
			channels |= (1 << io_timer->channel_index);
		}
	}

	return channels;
}

int io_timer_get_channel_mode(unsigned channel)
{
	if (channel >= MAX_IO_TIMERS)
	{
		return -1;
	}

	io_timers_t *io_timer = (io_timers_t *)(&io_timers[channel]);

	return (int)(io_timer->channel_mode);
}

int io_timer_free_channel(unsigned channel)
{
	int ret = io_timer_validate_channel_index(channel);

	if (ret != 0)
	{
		return ret;
	}

	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed)
	{
		io_timer_set_enable(false, IOTimerChanMode_NotUsed, 1 << channel);
	}

	return 0;
}

static int timer_set_rate(unsigned timer, unsigned rate)
{
	/* configure the timer to update at the desired rate */
	uint32_t cnt_sum = (AR_BUS_CLK / rate);

	cnt_sum = cnt_sum - 2;

	uint32_t cnt1 = get_timer_reg(timer, rTIM_LOADCOUNT);

	// DIABLE THE TIMER
	uint32_t tmp_data = get_timer_reg(timer, rTIM_CONTROLREG);
	tmp_data &= ~AR_TIMERCONTROLREG_ENABLE_ENABLE;
	set_timer_reg(timer, rTIM_CONTROLREG, tmp_data);

	// set the rate
	set_timer_reg(timer, rTIM_LOADCOUNT2, cnt_sum - cnt1);

	// ENABLE THE TIMER
	tmp_data |= AR_TIMERCONTROLREG_ENABLE_ENABLE;
	set_timer_reg(timer, rTIM_CONTROLREG, tmp_data);
	/* generate an update event; reloads the counter and all registers */
	return 0;
}

static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas OPE stops the Timer not the channel
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 8.1 Ms.
	 *  On 32 but timers this is 536870.912
	 */

	set_timer_reg(timer, rTIM_LOADCOUNT, 0xffffffff);
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	// set the pre scale freq for the timer  ar8020 need to do nothing;
	;
}

void io_timer_trigger(void)
{
	int oneshots = io_timer_get_mode_channels(IOTimerChanMode_OneShot);

	if (oneshots != 0)
	{
		// uint32_t action_cache[MAX_IO_TIMERS] = {0};
		// int actions = 0;

		/* Pre-calculate the list of timers to Trigger */

		// for (int timer = 0; timer < MAX_IO_TIMERS; timer++) {
		// 	if (validate_timer_index(timer) == 0) {
		// 		int channels = get_timer_channels(timer);

		// 		if (oneshots & channels) {
		// 			action_cache[actions++] = io_timers[timer].base;
		// 		}
		// 	}
		// }

		/* Now do them all with the shortest delay in between */

		irqstate_t flags = px4_enter_critical_section();

		// for (actions = 0; action_cache[actions] != 0 &&  actions < MAX_IO_TIMERS; actions++) {
		// 	_REG32(action_cache[actions], STM32_GTIM_EGR_OFFSET) |= GTIM_EGR_UG;
		// }

		px4_leave_critical_section(flags);
	}
}

int io_timer_init_timer(unsigned timer)
{
	io_timers_t *io_timer = (io_timers_t *)(&io_timers[timer]);

	/* Do this only once per timer */
	int rv = is_timer_uninitalized(timer);

	if (rv == 0)
	{
		irqstate_t flags = px4_enter_critical_section();

		set_timer_initalized(timer);

		/* enable the timer clock before we try to talk to it */
		// AR8020 need to do nothing;
		//modifyreg32(io_timers[timer].clock_register, 0, io_timers[timer].clock_bit);

		/* disable and configure the timer */

		set_timer_reg(timer, rTIM_CONTROLREG, AR_TIMERCONTROLREG_ENABLE_DISABLE);
		// set_timer_reg(timer, rTIM_LOADCOUNT, 10 - 1);
		// set_timer_reg(timer, rTIM_LOADCOUNT, 2560000 - 10 - 1); // default 50HZ

		get_timer_reg(timer, rTIM_EOI);

		set_timer_reg(timer, rTIM_CONTROLREG, AR_TIMERCONTROLREG_PWM_ENABLE | AR_TIMERCONTROLREG_INTMASK_MASKED | AR_TIMERCONTROLREG_MODE_USERDEFINED);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */
		timer_set_rate(timer, 50);

		/*
		 * Note that the timer is left disabled with IRQ subs installed
		 * and active but DEIR bits are not set.
		 */
		irq_attach(io_timers[timer].vectorno, io_timers[timer].handler, io_timer);

		up_enable_irq(io_timers[timer].vectorno);

		px4_leave_critical_section(flags);
	}

	return rv;
}

int io_timer_set_rate(unsigned timer, unsigned rate)
{
	int ret = io_timer_validate_channel_index(timer);

	if (ret != 0)
	{
		return ret;
	}

	io_timers_t *io_timer = (io_timers_t *)(&io_timers[timer]);

	if (rate == 0)
	{
		io_timer->channel_mode = IOTimerChanMode_OneShot;
		io_timer_set_oneshot_mode(timer);
	}
	else
	{
		timer_set_rate(timer, rate);
	}

	return OK;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
						  channel_handler_t channel_handler, void *context)
{

	io_timers_t *io_timer = (io_timers_t *)(&io_timers[channel]);
	io_timer->channel_mode = mode;
	io_timer->context = context;
	// io_timer->callback = channel_handler;

	uint32_t gpio = io_timer->gpio_out;
	uint32_t tmp;

	switch (mode)
	{
	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		tmp = get_timer_reg(channel, rTIM_CONTROLREG);
		tmp |= AR_TIMERCONTROLREG_PWM_ENABLE;
		set_timer_reg(channel, rTIM_CONTROLREG, tmp);
		break;

	case IOTimerChanMode_NotUsed:
		tmp = get_timer_reg(channel, rTIM_CONTROLREG);
		tmp &= ~AR_TIMERCONTROLREG_PWM_ENABLE;
		set_timer_reg(channel, rTIM_CONTROLREG, tmp);
		break;

	default:
		return -EINVAL;
	}

	/* Blindly try to initialize the timer - it will only do it once */
	io_timer_init_timer(channel);

	irqstate_t flags = px4_enter_critical_section();

	/* Set up IO */
	if (gpio)
	{
		px4_arch_configgpio(gpio);
	}

	px4_leave_critical_section(flags);

	return OK;
}

// enable
int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	io_timers_t *io_timer;

	irqstate_t flags = px4_enter_critical_section();

	for (uint8_t chan_index = 0; masks != 0 && chan_index < MAX_TIMER_IO_CHANNELS; chan_index++)
	{
		io_timer = (io_timers_t *)(&io_timers[chan_index]);

		if (masks & (1 << chan_index))
		{
			masks &= ~(1 << chan_index);

			uint32_t tmp_data = get_timer_reg(chan_index, rTIM_CONTROLREG);

			tmp_data |= state ? AR_TIMERCONTROLREG_ENABLE_ENABLE : AR_TIMERCONTROLREG_ENABLE_DISABLE;

			set_timer_reg(chan_index, rTIM_CONTROLREG, tmp_data);
			io_timer->channel_mode = mode;
		}
	}

	px4_leave_critical_section(flags);

	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);
	int mode = io_timer_get_channel_mode(channel);

	if (rv == 0)
	{
		if ((mode != IOTimerChanMode_PWMOut) &&
			(mode != IOTimerChanMode_OneShot) &&
			(mode != IOTimerChanMode_Trigger))
		{

			rv = -EIO;
		}
		else
		{

			uint32_t cnt2 = get_timer_reg(channels_timer(channel), rTIM_LOADCOUNT2);
 
			uint32_t cnt_value = value * 128;

			if(cnt2 != cnt_value)
			{

				//PX4_INFO("cnt2 = %d ; cnt_value = %d ", cnt2,cnt_value);
				uint32_t cnt1 = get_timer_reg(channels_timer(channel), rTIM_LOADCOUNT);
				
				/* configure the channel */
				uint32_t cnt_sum =  cnt1 + cnt2 ;

				set_timer_reg(channels_timer(channel), rTIM_LOADCOUNT2, cnt_value);
				set_timer_reg(channels_timer(channel), rTIM_LOADCOUNT, cnt_sum - cnt_value);
			
			}
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0)
	{
		int mode = io_timer_get_channel_mode(channel);

		if ((mode == IOTimerChanMode_PWMOut) ||
			(mode == IOTimerChanMode_OneShot) ||
			(mode == IOTimerChanMode_Trigger))
		{
			value = get_timer_reg(channels_timer(channel), rTIM_LOADCOUNT2) / 128;
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);
}
