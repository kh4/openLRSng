/*
 * OpenLRSng FRSKY compatible telemetry protocol
 *
 *  ----------------------------------------------------------------------
 * | HEADER | DataID1 | DATA1      | HEADER | DataID2 | DATA2      | TAIL |
 *  ----------------------------------------------------------------------
 * | 0x5E   |   ...   | LOW | HIGH | 0x5E   |   ...   | LOW | HIGH | 0x5E |
 *  ----------------------------------------------------------------------
 *
 *	The frame starts with 0x5E and ends with 0x5E, data for each sensor is separated by 0x5E.
 *
 *	--- DATAID
 *	1 - All data is sent in Little Endian except for latitude, longitude, voltage, date and time.
 *	1.1 - Latitude & longitude are separated by "." into 2 bytes.
 *	1.2 - Date is formated as DD/MM/YY
 *	1.3 - Time is formated as HH/MM/SS
 *
 *	--- DATA
 *	1 - If one of the following bytes are found on DATA..
 *	1.1 - 0x05E: Replace it with 0x5D 0x3E (two bytes)
 *	1.2 - 0x05D: Replace it with 0x5D 0x3D (two bytes)
 *	2 - When byte 0x5D is received, discard this byte, and the next byte is XORed with 0x60.
 *	
 *	-- TYPES OF FRAME
 *	1 - FRAME1 is sent per 200ms
 *	2 - FRAME2 is sent per 1000ms
 *	3 - FRAME3 is sent per 5000ms
 *
 */