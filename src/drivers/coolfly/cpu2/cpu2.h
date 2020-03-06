#ifndef COOLFLY_CPU2_H_
#define COOLFLY_CPU2_H_

#include <stddef.h>
#include <stdint.h>
#include <chip/ar_config.h>


#define SYS_EVENT_LEVEL_HIGH_MASK        0x10000000
#define SYS_EVENT_LEVEL_MIDIUM_MASK      0x20000000
#define SYS_EVENT_LEVEL_LOW_MASK         0x40000000
#define SYS_EVENT_INTER_CORE_MASK        0x80000000

#define SRAM_INTERCORE_MSG_LENGTH        64
#define SRAM_INTERCORE_EVENT_MAX_COUNT   (SRAM_INTERCORE_EVENT_CPU2T0_ST_SIZE / sizeof(AR_INTERCORE_EVENT))

typedef struct {
	uint16_t length;
	uint16_t seq;
	char data[SRAM_INTERCORE_MSG_LENGTH];
	uint32_t type;
	uint32_t isUsed;
} AR_INTERCORE_EVENT;

#define SYS_EVENT_ID_CPU2_LOG                        (SYS_EVENT_LEVEL_MIDIUM_MASK   | 0x001F)

#define SYS_EVENT_ID_READ_REG                       (SYS_EVENT_LEVEL_MIDIUM_MASK   | 0x0040)


#endif