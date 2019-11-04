#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "Mavlink.h"
#include <px4_config.h>
#include <sys/types.h>
#include <systemlib/err.h>
#include <drivers/device/i2c.h>
#include <nuttx/irq.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include <drivers/drv_intercore.h>

#include <uORB/uORB.h>
#include <uORB/topics/h264_input_format.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_parameter_map.h>

// Mavlink *Mavlink::_instance = nullptr;


#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

void crc_init(uint16_t* crcAccum);
void mavlink_start_checksum(mavlink_message_t* msg);
void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c);
void _mav_parse_error(mavlink_status_t *status);

void handle_input_rc(mavlink_message_t *msg);
void handle_rc_parameter_map();



#define _MAV_PAYLOAD_NON_CONST(msg) ((char *)(&((msg)->payload64[0])))


_EXT_ITCM void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	/*Accumulate one byte of data into the CRC*/
	uint8_t tmp;

	tmp = data ^ (uint8_t)(*crcAccum &0xff);
	tmp ^= (tmp<<4);
	*crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}


_EXT_ITCM void crc_init(uint16_t* crcAccum)
{
	*crcAccum = X25_INIT_CRC;
}


_EXT_ITCM uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
	uint16_t crcTmp;

	crc_init(&crcTmp);

	while (length--) 
	{
		crc_accumulate(*pBuffer++, &crcTmp);
	}

	return crcTmp;
}

_EXT_ITCM void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;

	while (length--) 
	{
	    crc_accumulate(*p++, crcAccum);
    }
}

_EXT_ITCM void mavlink_start_checksum(mavlink_message_t* msg)
{
	crc_init(&msg->checksum);
}

_EXT_ITCM void mavlink_update_checksum(mavlink_message_t* msg, uint8_t c)
{
	crc_accumulate(c, &msg->checksum);
}

_EXT_ITCM void _mav_parse_error(mavlink_status_t *status)
{
    status->parse_error++;
}

_EXT_ITCM uint8_t mavlink_parse_char(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char_buffer(r_message, r_mavlink_status, c);

    if (msg_received == MAVLINK_FRAMING_BAD_CRC || msg_received == MAVLINK_FRAMING_BAD_SIGNATURE) 
	{
	    r_mavlink_status->msg_received = MAVLINK_FRAMING_INCOMPLETE;
	    r_mavlink_status->parse_state = MAVLINK_PARSE_STATE_IDLE;

	    if (c == MAVLINK_STX)
	    {
		    r_mavlink_status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
		    r_message->len = 0;
		    mavlink_start_checksum(r_message);
	    }

	    return 0;
    }
    return msg_received;
}

_EXT_ITCM void handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case 65:
		// handle_rc_parameter_map();
		handle_input_rc(msg);
		break;

	default:
		break;
	}
}


#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))

#define _MAV_MSG_RETURN_TYPE(TYPE)  \
_EXT_ITCM static inline TYPE _MAV_RETURN_## TYPE(const mavlink_message_t *msg, uint8_t ofs) \
{ return *(const TYPE *)(&_MAV_PAYLOAD(msg)[ofs]);}

_MAV_MSG_RETURN_TYPE(uint8_t)
_MAV_MSG_RETURN_TYPE(uint16_t)
_MAV_MSG_RETURN_TYPE(int16_t)
_MAV_MSG_RETURN_TYPE(uint32_t)
_MAV_MSG_RETURN_TYPE(int32_t)
_MAV_MSG_RETURN_TYPE(uint64_t)
_MAV_MSG_RETURN_TYPE(int64_t)
_MAV_MSG_RETURN_TYPE(float)
_MAV_MSG_RETURN_TYPE(double)

#define _MAV_RETURN_ARRAY(TYPE, V)					\
_EXT_ITCM static inline uint16_t _MAV_RETURN_## TYPE ##_array(const mavlink_message_t *msg, TYPE *value, \
							 uint8_t array_length, uint8_t wire_offset) \
{ \
	memcpy(value, &_MAV_PAYLOAD(msg)[wire_offset], array_length*sizeof(TYPE)); \
	return array_length*sizeof(TYPE); \
}

_MAV_RETURN_ARRAY(uint8_t, u8)
_MAV_RETURN_ARRAY(uint16_t, u16)
_MAV_RETURN_ARRAY(uint32_t, u32)
_MAV_RETURN_ARRAY(uint64_t, u64)
_MAV_RETURN_ARRAY(int16_t,  i16)
_MAV_RETURN_ARRAY(int32_t,  i32)
_MAV_RETURN_ARRAY(int64_t,  i64)
_MAV_RETURN_ARRAY(float,    f)
_MAV_RETURN_ARRAY(double,   d)

_EXT_ITCM void handle_input_rc(mavlink_message_t *msg)
{
	static orb_advert_t input_rc_topic = NULL;
	static struct input_rc_s att;

	if (input_rc_topic == NULL)
	{
		input_rc_topic = orb_advertise(ORB_ID(input_rc), &att);
	}

	att.timestamp_last_signal = _MAV_RETURN_uint64_t(msg, 0);
	att.channel_count = _MAV_RETURN_uint32_t(msg, 8);
	att.rssi = _MAV_RETURN_uint32_t(msg, 12);
	att.rc_lost_frame_count = _MAV_RETURN_uint16_t(msg, 16);
	att.rc_total_frame_count = _MAV_RETURN_uint16_t(msg, 18);
	att.rc_ppm_frame_length = _MAV_RETURN_uint16_t(msg, 20);
	_MAV_RETURN_uint16_t_array(msg, att.values, 18,  22);
	att.rc_failsafe = _MAV_RETURN_uint8_t(msg, 58);
	att.rc_lost = _MAV_RETURN_uint8_t(msg, 59);
	att.input_source = _MAV_RETURN_uint8_t(msg, 60);
	_MAV_RETURN_uint8_t_array(msg, att._padding0, 3, 61);

	int instance = 0;
	orb_publish_auto(ORB_ID(input_rc), &input_rc_topic, &att, &instance, ORB_PRIO_HIGH);
	
	// orb_publish(ORB_ID(input_rc), input_rc_topic, &att);
}

// stable Value 
_EXT_ITCM void handle_rc_parameter_map()
{
	static orb_advert_t rc_parameter_map_topic = NULL;
	static struct rc_parameter_map_s att;

	if (rc_parameter_map_topic == NULL)
	{
		rc_parameter_map_topic = orb_advertise(ORB_ID(rc_parameter_map), &att);
	}

	for (size_t i = 0; i < 3; i++)
	{
		att.param_index[i] = i;
		att.scale[i] = 0;
		att.value0[i] = 0;
		att.value_min[i] = 0;
		att.value_max[i] = 512;
		att.valid[i]= 1;	
	}
	
	orb_publish(ORB_ID(rc_parameter_map), rc_parameter_map_topic, &att);
	
}



// use mavlink2 auto
_EXT_ITCM uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg,
									mavlink_status_t* status,
										uint8_t c)
{
	status->msg_received = MAVLINK_FRAMING_INCOMPLETE;

	switch (status->parse_state)
	{
	case MAVLINK_PARSE_STATE_UNINIT:
	case MAVLINK_PARSE_STATE_IDLE:
		// PX4_INFO("MAVLINK_PARSE_STATE_IDLE %02x \r\n", c);
		if (c == MAVLINK_STX)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;

			mavlink_start_checksum(rxmsg);
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_STX:

		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_STX %02x, msg_receive = %d \r\n", c, status->msg_received);

        if (status->msg_received != 0)
        {
            status->buffer_overrun++;
            _mav_parse_error(status);
            status->msg_received = 0;
            status->parse_state = MAVLINK_PARSE_STATE_IDLE;
        }
        else
        {
            // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2					
            rxmsg->len = c;
            status->packet_idx = 0;
            mavlink_update_checksum(rxmsg, c);

            status->parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;

        }
        break;

	case MAVLINK_PARSE_STATE_GOT_LENGTH:

		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_LENGTH %02x \r\n", c);

		rxmsg->incompat_flags = c;

		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS;

		break;

	case MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS %02x \r\n", c);

		rxmsg->compat_flags = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS;

		break;

	case MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS %02x \r\n", c);

		rxmsg->seq = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;

		break;

	case MAVLINK_PARSE_STATE_GOT_SEQ:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_SEQ %02x \r\n", c);

		rxmsg->sysid = c;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;

		break;

	case MAVLINK_PARSE_STATE_GOT_SYSID:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_SYSID %02x \r\n", c);

		rxmsg->compid = c;
		mavlink_update_checksum(rxmsg, c);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;

		break;

	case MAVLINK_PARSE_STATE_GOT_COMPID:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_COMPID %02x \r\n", c);

		rxmsg->msgid = c;
		mavlink_update_checksum(rxmsg, c);
        status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID1;

		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID1:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_MSGID1 %02x \r\n", c);

		rxmsg->msgid |= c<<8;
		mavlink_update_checksum(rxmsg, c);
		status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID2;
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID2:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_MSGID2 %02x \r\n", c);

		rxmsg->msgid |= ((uint32_t)c)<<16;
		mavlink_update_checksum(rxmsg, c);

		if(rxmsg->len > 0){
			status->parse_state = MAVLINK_PARSE_STATE_GOT_MSGID3;
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case MAVLINK_PARSE_STATE_GOT_MSGID3:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_MSGID3 %02x \r\n", c);

		_MAV_PAYLOAD_NON_CONST(rxmsg)[status->packet_idx++] = (char)c;
		mavlink_update_checksum(rxmsg, c);

		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		} 
		break;

	case MAVLINK_PARSE_STATE_GOT_PAYLOAD: 
    {
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_PAYLOAD %02x \r\n", c);
		
        // const mavlink_msg_entry_t *e = mavlink_get_msg_entry(rxmsg->msgid);
		// uint8_t crc_extra = e?e->crc_extra:0;
		uint8_t crc_extra = 0;

		mavlink_update_checksum(rxmsg, crc_extra);

		// PX4_INFO("checkSum = %x", rxmsg->checksum);

		if (c != (rxmsg->checksum & 0xFF)) {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_BAD_CRC1;
		} else {
			status->parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
		}

        rxmsg->ck[0] = c;
		break;
    }

	case MAVLINK_PARSE_STATE_GOT_CRC1:
	case MAVLINK_PARSE_STATE_GOT_BAD_CRC1:
		// PX4_INFO("MAVLINK_PARSE_STATE_GOT_CRC1 %02x \r\n", c);
	
		if (status->parse_state == MAVLINK_PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8)) {
			// got a bad CRC message
			status->msg_received = MAVLINK_FRAMING_BAD_CRC;
		} else {
			// Successfully got message
			status->msg_received = MAVLINK_FRAMING_OK;
		}
		rxmsg->ck[1] = c;

		break;

	case MAVLINK_PARSE_STATE_SIGNATURE_WAIT:
		// PX4_INFO("MAVLINK_PARSE_STATE_SIGNATURE_WAIT %02x \r\n", c);

		// not use signature 
		break;
    default:
        break;
	}

	if (status->msg_received == MAVLINK_FRAMING_OK)
	{
		status->current_rx_seq = rxmsg->seq;
		// Initial condition: If no packet has been received so far, drop count is undefined
		if (status->packet_rx_success_count == 0) status->packet_rx_drop_count = 0;
		// Count this packet as received
		status->packet_rx_success_count++;
	}

	// r_message->len = rxmsg->len; // Provide visibility on how far we are into current msg
	status->parse_error = 0;

	if (status->msg_received == MAVLINK_FRAMING_BAD_CRC) {
		/*
		  the CRC came out wrong. We now need to overwrite the
		  msg CRC with the one on the wire so that if the
		  caller decides to forward the message anyway that
		  mavlink_msg_to_send_buffer() won't overwrite the
		  checksum
		 */
		// r_message->checksum = rxmsg->ck[0] | (rxmsg->ck[1]<<8);
	}

	return status->msg_received;
}

