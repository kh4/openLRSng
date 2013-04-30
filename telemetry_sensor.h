 /*
  * openLRSng sensor related classes
  * 
  */
  
class telemetry_sensor {     // Currently there is nothing in common between sensor classes
	protected:               // so this only acts as a placeholder for future usage.
	public:
};
  
class telemetry_sensor_rssi: public telemetry_sensor {
	public:
		telemetry_type<uint8_t> tx;    // Each telemetry value holder is based on a class because there are
		telemetry_type<uint8_t> drop;  // special functions associated with them, never use a uint8_t directly.
		
		bool updated( void ) {                                                  // Will return true if the value changed since the last call to updated()
			if( this->tx.updated() or this->drop.updated() ) { return true; }   // Checks if either of the value holders were changed and return true if
			else { return false; }                                              // any of them changed or returns false if nothing changed.
		}
		
		uint8_t* exportPacket( void ) {                                                         // exportPacket() is responsable for exporting the frame with the correct structure
			uint8_t* data = (uint8_t *) malloc( sizeof(uint8_t) * (TELEMETRY_RSSI_PKTSZ +1) );  // Allocate the necessary memory
			data[0] = TELEMETRY_RSSI;                                                           // Sets the frame header
			data[1] = this->tx.get();                                                           // The second byte is the RX RSSI
			data[2] = this->drop.get();                                                         // The third byte is the total lost packets (more or less a dummy value for now)
			return data;                                                                        // Return the packet to caller, remember that memory deallocation is caller's responsability !
		}
		
		void importPacket( uint8_t* data ) {            // Populates the value holders with received data
			if( data[0] != TELEMETRY_RSSI ) { return; } // Sneaky sanity check based on header
			this->tx.set( data[1] );                    // As on the previous function, the second byte is the RX RSSI
			this->drop.set( data[2] );                  // and the third byte is the dummy value total lost packets.
		}
};