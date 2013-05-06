#ifndef _packet_h_included_
#define _packet_h_included_


class PacketBase
{
public:

};


class RxToTxPacket : public PacketBase
{
public:
	enum Constants
	{
		MaxRxToTx_DataLength = 28
	};

	// Flags
	// 
	enum MiscData
	{
		MiscData_ExtendData, // extending normal serial data with one byte. no use sending more misc stuff than what is shown in tx end. eg radio id mavlink packet.
		MiscData_LostPacketCount,
		MiscData_RemoteNoise, 
		MiscData_RemoteTxBuffer,
		MiscData_Debug,
		// .... up to 8 different things, with 3 bits (2^3).
	};

	// Content (of 'miscDataByte') could be varied through flags stuffed into dataLength byte.
	// 5 bits can specify datalength from 0-32, the 3 remaining bits can be used to distinguish
	// what miscDataByte means. And meaning of data can vary between each packet sent from Rx.
	// See 'MiscDataFlags' enum for a proposet dataset.
	uint8_t dataLength; // Length in bytes of accessory data stream (transparent serial link).
	uint8_t miscDataByte; 
	uint8_t data[MaxRxToTx_DataLength];
};


class TxToRxPacket : public PacketBase
{
public:
	enum Constants
	{
		MaxTxToRx_DataLength = 18
	};

	// 0xF5 = 0b11110101 => save failsafe
	// 0x5E = 0b01011110 => servo positions
	uint8_t packetFlags;

	// TODO: these fields below should be private, and set properly by accessor methods.
	// ppm ranges from 0-1023 (10 bits per ppm channel).
	uint8_t ppmLow0to3[4];
	uint8_t ppmHigh0to3; // 2 msb per ppm channel 0-4 

	uint8_t ppmLow4to7[4];
	uint8_t ppmHigh4to7; // 2 msb per ppm channel 0-4 

	// TODO: accessor methods for ppm channels into structure bytes
	uint8_t dataLength; // Length in bytes of accessory data stream (transparent serial link).
	uint8_t data[MaxTxToRx_DataLength];
};



#endif