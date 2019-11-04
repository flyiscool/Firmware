# AR8020 - the msg is used for firmware it66021.cpp to cpu2.cpp 
# origin type 
#
#   typedef struct _SysEvent_H264InputFormatChangeParameter
#   {
#       uint8_t  index;
#       uint16_t width;
#       uint16_t hight;
#       uint8_t  framerate;
#       uint8_t  vic;
#       ENUM_ENCODER_INPUT_SRC  e_h264InputSrc;
#       uint8_t  reserve[SYS_EVENT_HANDLER_PARAMETER_LENGTH - 9];
#   } STRU_SysEvent_H264InputFormatChangeParameter;

#   typedef enum
#   {
#       ENCODER_INPUT_SRC_HDMI_0 = 1,
#       ENCODER_INPUT_SRC_HDMI_1,
#       ENCODER_INPUT_SRC_DVP_0,
#       ENCODER_INPUT_SRC_DVP_1,
#       ENCODER_INPUT_SRC_MIPI,
#   } ENUM_ENCODER_INPUT_SRC;

#   #define SYS_EVENT_HANDLER_PARAMETER_LENGTH    16

uint8 index
uint16 width
uint16 hight
uint8 framerate
uint8 vic
uint8 e_h264InputSrc
uint8[7]  reserve




