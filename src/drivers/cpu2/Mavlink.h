#include <stdio.h>
#include <stdlib.h>

__BEGIN_DECLS

#ifndef MAVLINK_MAX_PAYLOAD_LEN
// it is possible to override this, but be careful!
#define MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length
#endif

#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and stx
#define MAVLINK_NUM_CHECKSUM_BYTES 2

#define MAVLINK_SIGNATURE_BLOCK_LEN 13

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

typedef enum {
    MAVLINK_PARSE_STATE_UNINIT=0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_INCOMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_COMPAT_FLAGS,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID1,
    MAVLINK_PARSE_STATE_GOT_MSGID2,
    MAVLINK_PARSE_STATE_GOT_MSGID3,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1,
    MAVLINK_PARSE_STATE_GOT_BAD_CRC1,
    MAVLINK_PARSE_STATE_SIGNATURE_WAIT
} mavlink_parse_state_t; ///< The state machine for the comm parser


typedef enum {
    MAVLINK_FRAMING_INCOMPLETE=0,
    MAVLINK_FRAMING_OK=1,
    MAVLINK_FRAMING_BAD_CRC=2,
    MAVLINK_FRAMING_BAD_SIGNATURE=3
} mavlink_framing_t;

typedef struct __mavlink_message {
	uint16_t checksum;      ///< sent at end of packet
	uint8_t magic;          ///< protocol magic marker
	uint8_t len;            ///< Length of payload
	uint8_t incompat_flags; ///< flags that must be understood
	uint8_t compat_flags;   ///< flags that can be ignored if not understood
	uint8_t seq;            ///< Sequence of packet
	uint8_t sysid;          ///< ID of message sender system/aircraft
	uint8_t compid;         ///< ID of the message sender component
	uint32_t msgid:24;      ///< ID of message in payload
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
	uint8_t ck[2];          ///< incoming checksum bytes
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
} mavlink_message_t;

typedef struct __mavlink_status {
    uint8_t msg_received;               ///< Number of received messages
    uint8_t buffer_overrun;             ///< Number of buffer overruns
    uint8_t parse_error;                ///< Number of parse errors
    mavlink_parse_state_t parse_state;  ///< Parsing state machine
    uint8_t packet_idx;                 ///< Index in current packet
    uint8_t current_rx_seq;             ///< Sequence number of last packet received
    uint8_t current_tx_seq;             ///< Sequence number of last packet sent
    uint16_t packet_rx_success_count;   ///< Received packets
    uint16_t packet_rx_drop_count;      ///< Number of packet drops
    uint8_t flags;                      ///< MAVLINK_STATUS_FLAG_*
    uint8_t signature_wait;             ///< number of signature bytes left to receive
    struct __mavlink_signing *signing;  ///< optional signing state
    struct __mavlink_signing_streams *signing_streams; ///< global record of stream timestamps
} mavlink_status_t;


// class Mavlink
// {
// public:
//     static void initialize();

//     static Mavlink &instance();

//     static void handle_message(mavlink_message_t *msg);

//     uint8_t mavlink_frame_char_buffer(uint8_t c);
    
// private:
//     mavlink_message_t msg;
//     mavlink_status_t status;

//     static Mavlink *_instance;

//     Mavlink();
// }


uint8_t mavlink_frame_char_buffer(mavlink_message_t* rxmsg,
                                                mavlink_status_t* status,
                                                uint8_t c);

void handle_message(mavlink_message_t *msg);

uint8_t mavlink_parse_char(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status);

uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);

void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);

void crc_accumulate(uint8_t data, uint16_t *crcAccum);


__END_DECLS