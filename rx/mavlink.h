#ifndef _Mavlink_Included_
#define _Mavlink_Included_

struct mavlink_RADIO_v10 {
	uint16_t rxerrors;
	uint16_t fixed;
	uint8_t rssi;
	uint8_t remrssi;
	uint8_t txbuf;
	uint8_t noise;
	uint8_t remnoise;
};

// use '3D' for 3DRadio
#define RADIO_SOURCE_SYSTEM '3'
#define RADIO_SOURCE_COMPONENT 'D'

#define MAVLINK_MSG_ID_RADIO 166
#define MAVLINK_RADIO_CRC_EXTRA 21
#define MAV_HEADER_SIZE 6
#define MAV_MAX_PACKET_LENGTH  (MAV_HEADER_SIZE + sizeof(struct mavlink_RADIO_v10) + 2)

extern uint8_t g_mavlinkBuffer[MAV_MAX_PACKET_LENGTH];

#endif 
