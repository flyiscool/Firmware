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
#pragma once

#include <px4_defines.h>

#include <stdint.h>
#include <sys/ioctl.h>

__BEGIN_DECLS

#define SYS_EVENT_HANDLER_PARAMETER_LENGTH    16



#define SYS_EVENT_ID_IDLE (SYS_EVENT_LEVEL_LOW_MASK | 0x8000)

// Misc driver event
#define SYS_EVENT_ID_H264_INPUT_FORMAT_CHANGE         (SYS_EVENT_LEVEL_MIDIUM_MASK | 0x0001)
#define SYS_EVENT_ID_BB_SUPPORT_BR_CHANGE             (SYS_EVENT_LEVEL_MIDIUM_MASK | 0x0002)
#define SYS_EVENT_ID_BB_EVENT                         (SYS_EVENT_LEVEL_MIDIUM_MASK | 0x0003)
#define SYS_EVENT_ID_USER_CFG_CHANGE                  (SYS_EVENT_LEVEL_HIGH_MASK   | 0x0004)
#define SYS_EVENT_ID_USB_PLUG_OUT                     (SYS_EVENT_LEVEL_HIGH_MASK   | 0x0005)
#define SYS_EVENT_ID_NV_MSG                           (SYS_EVENT_LEVEL_MIDIUM_MASK | 0x0006)
#define SYS_EVENT_ID_USB_SWITCH_HOST_DEVICE           (SYS_EVENT_LEVEL_HIGH_MASK   | 0x0007)


typedef enum
{
    ENCODER_INPUT_SRC_HDMI_0 = 1,
    ENCODER_INPUT_SRC_HDMI_1,
    ENCODER_INPUT_SRC_DVP_0,
    ENCODER_INPUT_SRC_DVP_1,
    ENCODER_INPUT_SRC_MIPI,
} ENUM_ENCODER_INPUT_SRC;


typedef struct _SysEvent_H264InputFormatChangeParameter
{
    uint8_t  index;
    uint16_t width;
    uint16_t hight;
    uint8_t  framerate;
    uint8_t  vic;
    ENUM_ENCODER_INPUT_SRC  e_h264InputSrc;
    uint8_t  reserve[SYS_EVENT_HANDLER_PARAMETER_LENGTH - 9];
} STRU_SysEvent_H264InputFormatChangeParameter;


__END_DECLS

