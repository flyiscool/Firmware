#ifndef _COOLFLY_CPU2_H_
#define _COOLFLY_CPU2_H_

#include <stddef.h>
#include <stdint.h>

#include <chip/ar_config.h>

#define SYS_EVENT_LEVEL_HIGH_MASK             0x10000000
#define SYS_EVENT_LEVEL_MIDIUM_MASK           0x20000000
#define SYS_EVENT_LEVEL_LOW_MASK              0x40000000
#define SYS_EVENT_INTER_CORE_MASK             0x80000000


#define SRAM_INTERCORE_MSG_LENGTH           64
#define SRAM_INTERCORE_EVENT_MAX_COUNT            (SRAM_INTERCORE_EVENT_CPU2T0_ST_SIZE / sizeof(AR_INTERCORE_EVENT))

typedef struct
{
    uint16_t             length;
    uint16_t             seq;
    char                data[SRAM_INTERCORE_MSG_LENGTH];
    uint32_t            type;
    uint32_t             isUsed;
} AR_INTERCORE_EVENT;


typedef struct _AR_INTERCORE_EVENT_NODE
{
    AR_INTERCORE_EVENT message;
    
    struct _AR_INTERCORE_EVENT_NODE *next;

} AR_INTERCORE_EVENT_NODE;


typedef struct 
{
    AR_INTERCORE_EVENT_NODE *firstNode;
    
    AR_INTERCORE_EVENT_NODE *lastNode;

    uint32_t node_count;

    uint32_t max_node_count;

} AR_INTERCORE_EVENT_NODE_LIST;

// cpu0 to cpu2 communication 
// static init_timer_st TIM2_CH0_SYSEVENT = 
// {
//     .base_time_group = 2,
//     .time_num = 0,
//     .ctrl = TIME_ENABLE | USER_DEFINED,
// };

// static init_timer_st TIM2_CH1_SYSEVENT = 
// {
//     .base_time_group = 2,
//     .time_num = 1,
//     .ctrl = TIME_ENABLE | USER_DEFINED,
// };

// //
// static init_timer_st TIM2_CH2_SYSEVENT = 
// {
//     .base_time_group = 2,
//     .time_num = 2,
//     .ctrl = TIME_ENABLE | USER_DEFINED,
// };

// cpu2 to cpu0 communication 
// static init_timer_st TIM2_CH3_SYSEVENT = 
// {
//     .base_time_group = 2,
//     .time_num = 3,
//     .ctrl = TIME_ENABLE | USER_DEFINED,
// };

// static init_timer_st TIM2_CH4_SYSEVENT = 
// {
//     .base_time_group = 2,
//     .time_num = 4,
//     .ctrl = TIME_ENABLE | USER_DEFINED,
// };


#define SYS_EVENT_ID_CPU2_LOG                        (SYS_EVENT_LEVEL_MIDIUM_MASK   | 0x001F)

#define SYS_EVENT_ID_READ_REG                       (SYS_EVENT_LEVEL_MIDIUM_MASK   | 0x0040)


#endif