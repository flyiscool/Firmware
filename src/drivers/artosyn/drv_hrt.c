/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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

/**
 * @file drv_hrt.c
 *
 * High-resolution timer callouts and timekeeping.
 *
 * This can use any general or advanced STM32 timer.
 *
 * Note that really, this could use systick too, but that's
 * monopolised by NuttX and stealing it would just be awkward.
 *
 * We don't use the NuttX Artosyn driver per se; rather, we
 * claim the timer and then drive it directly.
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

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include "ar_uart.h"
#include "ar_gpio.h"
#include "ar_tim.h"
#include "chip/ar_tim.h"
#include "chip/ar_config.h"

#ifdef CONFIG_DEBUG_HRT
#  define hrtinfo _info
#else
#  define hrtinfo(x...)
#endif

#if defined(HRT_TIMER_FREERUN) && defined(HRT_TIMER_CCR)

# define HRT_TIMER_CLOCK        		AR_BUS_CLK

/*
 * HRT clock must be a multiple of 1MHz & greater than 100MHz
 */
// #if (HRT_TIMER_CLOCK % 100000000) != 0
// 	# error HRT_TIMER_CLOCK must be a multiple of 100MHz  !! liuwei
// #endif
// #if HRT_TIMER_CLOCK <  100000000
// 	# error HRT_TIMER_CLOCK must be greater than 100MHz  !! liuwei
// #endif


/* freerun timer */
#if HRT_TIMER_FREERUN == 0
	# define HRT_TIMER_FREERUN_BASE         AR_TIM0_BASE
#elif HRT_TIMER_FREERUN == 1
	# define HRT_TIMER_FREERUN_BASE         AR_TIM1_BASE
#elif   HRT_TIMER_FREERUN == 2
	# define HRT_TIMER_FREERUN_BASE         AR_TIM2_BASE
#endif

/* freerun timer channel */
#if HRT_TIMER_FREERUN == 0
	#if HRT_TIMER_FREERUN_CHANNEL == 0
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH0
	#elif   HRT_TIMER_FREERUN_CHANNEL == 1
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH1
	#elif   HRT_TIMER_FREERUN_CHANNEL == 2
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH2
	#elif   HRT_TIMER_FREERUN_CHANNEL == 3
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH3
	#elif   HRT_TIMER_FREERUN_CHANNEL == 4
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH4
	#elif   HRT_TIMER_FREERUN_CHANNEL == 5
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH5
	#elif   HRT_TIMER_FREERUN_CHANNEL == 6
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH6
	#elif   HRT_TIMER_FREERUN_CHANNEL == 7
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM0_CH7
	#endif
#elif HRT_TIMER_FREERUN == 1
	#if HRT_TIMER_FREERUN_CHANNEL == 0
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH0
	#elif   HRT_TIMER_FREERUN_CHANNEL == 1					 
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH1
	#elif   HRT_TIMER_FREERUN_CHANNEL == 2					
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH2
	#elif   HRT_TIMER_FREERUN_CHANNEL == 3					
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH3
	#elif   HRT_TIMER_FREERUN_CHANNEL == 4					
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH4
	#elif   HRT_TIMER_FREERUN_CHANNEL == 5					
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH5
	#elif   HRT_TIMER_FREERUN_CHANNEL == 6					
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH6
	#elif   HRT_TIMER_FREERUN_CHANNEL == 7					
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM1_CH7
	#endif
#elif   HRT_TIMER_FREERUN == 2
	#if HRT_TIMER_FREERUN_CHANNEL == 0
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH0
	#elif   HRT_TIMER_FREERUN_CHANNEL == 1                      	
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH1
	#elif   HRT_TIMER_FREERUN_CHANNEL == 2                      
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH2
	#elif   HRT_TIMER_FREERUN_CHANNEL == 3                      
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH3
	#elif   HRT_TIMER_FREERUN_CHANNEL == 4                      
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH4
	#elif   HRT_TIMER_FREERUN_CHANNEL == 5                      
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH5
	#elif   HRT_TIMER_FREERUN_CHANNEL == 6                      
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH6
	#elif   HRT_TIMER_FREERUN_CHANNEL == 7                      
	# 	define HRT_TIMER_FREERUN_VECTOR       AR_IRQ_TIM2_CH7
	#endif
#endif


/* compare timer */
#if HRT_TIMER_CCR == 0
	# define HRT_TIMER_CCR_BASE         AR_TIM0_BASE
#elif HRT_TIMER_CCR == 1
	# define HRT_TIMER_CCR_BASE         AR_TIM1_BASE
#elif   HRT_TIMER_CCR == 2
	# define HRT_TIMER_CCR_BASE         AR_TIM2_BASE
#endif


/* compare timer channel */
#if HRT_TIMER_CCR == 0
	#if HRT_TIMER_CCR_CHANNEL == 0
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH0
	#elif   HRT_TIMER_CCR_CHANNEL == 1
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH1
	#elif   HRT_TIMER_CCR_CHANNEL == 2
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH2
	#elif   HRT_TIMER_CCR_CHANNEL == 3
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH3
	#elif   HRT_TIMER_CCR_CHANNEL == 4
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH4
	#elif   HRT_TIMER_CCR_CHANNEL == 5
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH5
	#elif   HRT_TIMER_CCR_CHANNEL == 6
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH6
	#elif   HRT_TIMER_CCR_CHANNEL == 7
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM0_CH7
	#endif
#elif HRT_TIMER_CCR == 1
	#if HRT_TIMER_CCR_CHANNEL == 0
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH0
	#elif   HRT_TIMER_CCR_CHANNEL == 1					 
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH1
	#elif   HRT_TIMER_CCR_CHANNEL == 2					
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH2
	#elif   HRT_TIMER_CCR_CHANNEL == 3					
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH3
	#elif   HRT_TIMER_CCR_CHANNEL == 4					
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH4
	#elif   HRT_TIMER_CCR_CHANNEL == 5					
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH5
	#elif   HRT_TIMER_CCR_CHANNEL == 6					
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH6
	#elif   HRT_TIMER_CCR_CHANNEL == 7					
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM1_CH7
	#endif
#elif   HRT_TIMER_CCR == 2
	#if HRT_TIMER_CCR_CHANNEL == 0
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH0
	#elif   HRT_TIMER_CCR_CHANNEL == 1                      	
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH1
	#elif   HRT_TIMER_CCR_CHANNEL == 2                      
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH2
	#elif   HRT_TIMER_CCR_CHANNEL == 3                      
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH3
	#elif   HRT_TIMER_CCR_CHANNEL == 4                      
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH4
	#elif   HRT_TIMER_CCR_CHANNEL == 5                      
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH5
	#elif   HRT_TIMER_CCR_CHANNEL == 6                      
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH6
	#elif   HRT_TIMER_CCR_CHANNEL == 7                      
	# 	define HRT_TIMER_CCR_VECTOR       AR_IRQ_TIM2_CH7
	#endif
#endif


/**
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 32-bit timer/counter clocked
 * at 100MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	50000

/*
 * Period of the free-running counter, in microseconds.
 */
//#define HRT_COUNTER_PERIOD	0xFFFFFFFF/(CPU0_CPU1_CORE_PLL_CLK/1000000/2)
#define HRT_COUNTER_PERIOD	0xFFFFFFFF

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 *  100MHz  so  / 100
 */
#define HRT_COUNTER_SCALE(_c)	(_c/(CPU0_CPU1_CORE_PLL_CLK/1000000/2))

/*
 * Timer register accessors
 */

#define REG_FR(_reg)       	(HRT_TIMER_FREERUN_BASE + _reg)
 

#if HRT_TIMER_FREERUN_CHANNEL == 0
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER0LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER0LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER0CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER0CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER0EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER0INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 1
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER1LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER1LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER1CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER1CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER1EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER1INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 2
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER2LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER2LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER2CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER2CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER2EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER2INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 3
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER3LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER3LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER3CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER3CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER3EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER3INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 4
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER4LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER4LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER4CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER4CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER4EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER4INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 5
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER5LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER5LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER5CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER5CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER5EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER5INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 6
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER6LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER6LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER6CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER6CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER6EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER6INTSTATUS_OFFSET   )
#elif   HRT_TIMER_FREERUN_CHANNEL == 7
#   define rTIMERLOADCOUNT_HRT_FR                REG_FR(AR_TIM_TIMER7LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_FR               REG_FR(AR_TIM_TIMER7LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_FR             REG_FR(AR_TIM_TIMER7CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_FR               REG_FR(AR_TIM_TIMER7CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_FR                      REG_FR(AR_TIM_TIMER7EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_FR                REG_FR(AR_TIM_TIMER7INTSTATUS_OFFSET   )
#endif

#define REG_CCR(_reg)       (HRT_TIMER_CCR_BASE + _reg)

#if HRT_TIMER_CCR_CHANNEL == 0
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER0LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER0LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER0CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER0CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER0EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER0INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 1
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER1LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER1LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER1CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER1CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER1EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER1INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 2
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER2LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER2LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER2CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER2CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER2EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER2INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 3
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER3LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER3LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER3CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER3CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER3EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER3INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 4
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER4LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER4LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER4CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER4CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER4EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER4INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 5
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER5LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER5LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER5CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER5CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER5EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER5INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 6
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER6LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER6LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER6CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER6CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER6EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER6INTSTATUS_OFFSET   )
#elif   HRT_TIMER_CCR_CHANNEL == 7
#   define rTIMERLOADCOUNT_HRT_CCR                REG_CCR(AR_TIM_TIMER7LOADCOUNT_OFFSET   )
#   define rTIMERLOADCOUNT2_HRT_CCR               REG_CCR(AR_TIM_TIMER7LOADCOUNT2_OFFSET  )
#   define rTIMERCURRENTVALUE_HRT_CCR             REG_CCR(AR_TIM_TIMER7CURRENTVALUE_OFFSET)
#   define rTIMERCONTROLREG_HRT_CCR               REG_CCR(AR_TIM_TIMER7CONTROLREG_OFFSET  )
#   define rTIMEREOI_HRT_CCR                      REG_CCR(AR_TIM_TIMER7EOI_OFFSET         )
#   define rTIMERINTSTATUS_HRT_CCR                REG_CCR(AR_TIM_TIMER7INTSTATUS_OFFSET   )
#endif


/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint32_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint32_t			latency_actual;


/* latency histogram */
#define LATENCY_BUCKET_COUNT 8
__EXPORT const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
__EXPORT const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t		latency_counters[LATENCY_BUCKET_COUNT + 1];


/* timer-specific functions */
static void		hrt_tim_init(void);
static int		hrt_tim_ccr_isr(int irq, void *context, void *arg);
static void		hrt_latency_update(void);

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		                            hrt_abstime deadline,
		                            hrt_abstime interval,
		                            hrt_callout callout,
		                            void *arg);

static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);


/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
static void
hrt_tim_init(void)
{

	/* claim our interrupt vector */
	irq_attach(HRT_TIMER_CCR_VECTOR, hrt_tim_ccr_isr, NULL);
	
	/* clock/power on our timer */ 
	// ar8020 clock is power by default

	/*freerun timer*/
	putreg32(AR_TIMERCONTROLREG_ENABLE_DISABLE,rTIMERCONTROLREG_HRT_FR);

    /* clear cnt */
	putreg32(0xFFFFFFFF,rTIMERLOADCOUNT_HRT_FR);

    /* clear cnt2 */
	putreg32(0,rTIMERLOADCOUNT2_HRT_FR);

    getreg32(rTIMEREOI_HRT_FR);	/* clear inturput */

	putreg32(AR_TIMERCONTROLREG_PWM_DISABLE	|   \
							AR_TIMERCONTROLREG_INTMASK_MASKED 	|   \
							AR_TIMERCONTROLREG_MODE_FREERUN 	|   \
							AR_TIMERCONTROLREG_ENABLE_ENABLE,rTIMERCONTROLREG_HRT_FR);


	/*ccr timer*/
	putreg32(AR_TIMERCONTROLREG_ENABLE_DISABLE,rTIMERCONTROLREG_HRT_CCR);
	
    /* clear cnt */
	putreg32(128000-1,rTIMERLOADCOUNT_HRT_CCR);
	
	putreg32(0,rTIMERLOADCOUNT2_HRT_CCR);

    getreg32(rTIMEREOI_HRT_CCR);	/* clear inturput */

	putreg32(AR_TIMERCONTROLREG_PWM_DISABLE	|   \
							AR_TIMERCONTROLREG_INTMASK_NOMASKED 	|   \
							AR_TIMERCONTROLREG_MODE_USERDEFINED 		|   \
							AR_TIMERCONTROLREG_ENABLE_ENABLE,rTIMERCONTROLREG_HRT_CCR);

	/* enable interrupts */
	up_enable_irq(HRT_TIMER_CCR_VECTOR);
}

static bool flash =  true;

static uint16_t flashCnt = 0;

	//static uint8_t cnt = 0;
/**
 * Handle the compare interrupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int 
hrt_tim_ccr_isr(int irq, void *context, void *arg)
{
	if (flashCnt ++ == 4500)
	{
		flash = !flash;
		ar_gpiowrite(GPIO_OUTPUT|GPIO_OUTSET|GPIO_PIN14, flash); 

		flashCnt = 0;
	}
	
	/* grab the timer for latency tracking purposes */
	// because the ar8020 is only -1 cnt mode
		
	latency_actual = 0xFFFFFFFF - getreg32(rTIMERCURRENTVALUE_HRT_FR);
	
	/* ack the interrupts we just read */
	getreg32(rTIMEREOI_HRT_CCR);

	/* do latency calculations */
	hrt_latency_update();

	/* run any callouts that have met their deadline */
	hrt_call_invoke();

	/* and schedule the next interrupt */
	hrt_call_reschedule();
	
	return OK;
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime 
hrt_absolute_time(void)
{
	hrt_abstime	abstime;
	uint32_t	count;
	irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	static volatile hrt_abstime base_time;
	static volatile uint32_t last_count;

	/* prevent re-entry */
	flags = px4_enter_critical_section();

	/* get the current counter value */
	/* because the ar8020 is -1 cnt mode */ 
	count = (0xFFFFFFFF - getreg32(rTIMERCURRENTVALUE_HRT_FR));
	
	/*
	 * Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	
	if (count < last_count) 
	{
		base_time += 0xFFFFFFFF;
	}

	/* save the count for next time */
	last_count = count;

	/* compute the current time */
	//abstime = HRT_COUNTER_SCALE(base_time + count);
	abstime = (base_time + count) >> 7;

	px4_leave_critical_section(flags);

	return abstime;
}

/**
 * Convert a timespec to absolute time
 */
hrt_abstime
ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/**
 * Convert absolute time to a timespec.
 */
void
abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

/**
 * Compare a time value with the current time.
 */
hrt_abstime
hrt_elapsed_time(const volatile hrt_abstime *then)
{
	irqstate_t flags = px4_enter_critical_section();

	hrt_abstime delta = hrt_absolute_time() - *then;

	px4_leave_critical_section(flags);

	return delta;
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
hrt_abstime
hrt_store_absolute_time(volatile hrt_abstime *now)
{
	irqstate_t flags = px4_enter_critical_section();

	hrt_abstime ts = hrt_absolute_time();

	px4_leave_critical_section(flags);

	return ts;
}

/**
 * Initialise the high-resolution timing module.
 */
void
hrt_init(void)
{
	sq_init(&callout_queue);
	hrt_tim_init();

}

/**
 * Call callout(arg) after interval has elapsed.
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = px4_enter_critical_section();

	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0) {
		sq_rem(&entry->link, &callout_queue);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);

	px4_leave_critical_section(flags);
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = px4_enter_critical_section();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	px4_leave_critical_section(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline))
    {
		sq_addfirst(&entry->link, &callout_queue);
		hrtinfo("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} 
    else 
    {
		do 
        {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) 
            {
				hrtinfo("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	hrtinfo("scheduled\n");
}

static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	while (true) 
    {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL) 
        {
			break;
		}

		if (call->deadline > now) 
        {
			break;
		}

		sq_rem(&call->link, &callout_queue);
		hrtinfo("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) 
        {
			hrtinfo("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) 
        {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) 
            {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	uint32_t now_fr_cnt = 0xFFFFFFFF - getreg32(rTIMERCURRENTVALUE_HRT_FR);
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	/*
		* Determine what the next deadline will be.
		*
		* Note that we ensure that this will be within the counter
		* period, so that when we truncate all but the low 16 bits
		* the next time the compare matches it will be the deadline
		* we want.
		*
		* It is important for accurate timekeeping that the compare
		* interrupt fires sufficiently often that the base_time update in
		* hrt_absolute_time runs at least once per timer period.
		*/
	if (next != NULL) 
	{
		hrtinfo("entry in queue\n");

		if (next->deadline <= (now + HRT_INTERVAL_MIN)) 
		{
			hrtinfo("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;
		} 
		else if (next->deadline < deadline) 
		{
			hrtinfo("due soon\n");
			deadline = next->deadline;
		}
	}

	//hrtinfo("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	

	uint32_t temp = deadline -now;
	

	latency_baseline = now_fr_cnt + temp *128;

	// latency_baseline -= deadline * 128;
	putreg32(AR_TIMERCONTROLREG_ENABLE_DISABLE,rTIMERCONTROLREG_HRT_CCR);
	putreg32(temp *128, rTIMERLOADCOUNT_HRT_CCR);

	putreg32(AR_TIMERCONTROLREG_PWM_DISABLE	|   \
							AR_TIMERCONTROLREG_INTMASK_NOMASKED 	|   \
							AR_TIMERCONTROLREG_MODE_USERDEFINED 		|   \
							AR_TIMERCONTROLREG_ENABLE_ENABLE,rTIMERCONTROLREG_HRT_CCR);
}

static void
hrt_latency_update(void)
{
	uint32_t latency = (latency_actual - latency_baseline)/128;
	unsigned	index;
	
	/* bounded buckets */
	for (index = 0; index < LATENCY_BUCKET_COUNT; index++)
     {
		if (latency <= latency_buckets[index]) 
        {
			latency_counters[index]++;
			return;
		}
	}

	/* catch-all at the end */
	latency_counters[index]++;
}

void
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

#endif /* HRT_TIMER */
