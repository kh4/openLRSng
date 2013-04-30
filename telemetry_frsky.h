/*
 * OpenLRSng FRSKY compatible telemetry protocol
 *
 *  -----------------------------------------------------------
 * | HEADER | LINKPKT | VOLT1 | VOLT2 | TXRSSI | RXRSSI | TAIL |
 *  -----------------------------------------------------------
 * | 0x7E   | 0xFE    |       |       |        |        | 0x7E |
 *  -----------------------------------------------------------
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
 
class frsky_class : public telemetry_class {
	private:
		uint32_t telemetryFrsky1;
	
	protected:
		void writeSerial( uint8_t* data, int bytes ) {
			for( uint8_t i = 0; i < bytes; i++ ) { Serial.print( (char) data[i] ); }
			Serial.println(); // FRSKY expects a carriage return at the end
		}
	
	public:
		void tick( void ) {                                 // This function is responsable for forwarding the telemetry data
			if( millis() - this->telemetryFrsky1 > 190 ) {  // back to whatever is listening on the serial line, in this specific
				this->telemetryFrsky1 = millis();           // module we expect a FRSKY compatible receiver like OpenTX.
				this->forward();                            // Currently this only serves as an example sending the RSSI packet
			}
		}
		
		void forward( void ) {
			uint8_t data[11];
			data[0]  = 0x7e;	// Header
			data[1]  = 0xfe;	// Link packet flag
			data[2]  = 0;		// Voltage A1
			data[3]  = 0;		// Voltage A2
			data[4]  = map( this->rssi.tx.get(), 0, 255, 0, 100 ); // RX RSSI
			data[5]  = 0;		// TX RSSI
			data[6]  = 0x00;	// Packet padding
			data[7]  = 0x00;	//
			data[8]  = 0x00;	//
			data[9]  = 0x00;	//
			data[10] = 0x7e;	// Tail
			writeSerial( data, 11 );
		}
};

// TELEMETRY
frsky_class telemetry;