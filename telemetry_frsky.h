/*
 * OpenLRSng FRSKY compatible telemetry protocol
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
 
class tFrsky : public tCore {
	private:
		// nothing here
	
	protected:
		// nothing here
	
	public:
		void queue( uint8_t byte ) {
			uint8_t t_size = 11;
			uint8_t* t_pkt = (uint8_t *) malloc( t_size );
			
			t_pkt[0] = 0x7E; // header
			t_pkt[1] = 0xfe;  // LINKPKT
			t_pkt[2] = byte; // Analog value for port1
			t_pkt[3] = byte; // Analog value for port2
			t_pkt[4] = byte; // Up link quality
			t_pkt[5] = byte; // Down link quality
			t_pkt[6] = 0x00; // filler
			t_pkt[7] = 0x00; // filler
			t_pkt[8] = 0x00; // filler
			t_pkt[9] = 0x00; // filler
			t_pkt[10] = 0x7E; // tail
			
			this->tx_add( t_pkt, t_size );
			free( t_pkt ); // cleanup
		}
};

// TELEMETRY
tFrsky telemetry;