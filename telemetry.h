 /*
  * openLRSng telemetry module
  * 
  * //###### OPENLRSNG PROTOCOL #######
  * 0x00 - Channel values
  * 0x01 - Set failsafe for all channels
  * 0x40 to 0x7f - Telemetry data
  * 0xf0 to 0xff - Transparent data
  * 
  * //###### TELEMETRY PACKET #######
  * [FLAG][DATA1][DATA2][DATA3][DATA4][DATA5][DATA6][DATA7][DATA8]
  *
  * WARNING: The following table is subject to change and it's current purpose is to act as a early draft.
  * 
  * STATUS      | SENSOR TYPE       | FLAG | DATA0   | DATA1   | DATA2   | DATA3   | DATA4   | DATA5   | DATA6   | DATA7   | DATA8   | UNIT           | RANGE   |
  * ------------ ------------------- ------ --------- --------- --------- --------- --------- --------- --------- --------- --------- ---------------- --------- 
  * IN PROGRESS | TELEMETRY_RSSI    | 0x40 | RX_RSSI | RX_DROP |         |         |         |         |         |         |         | N/A            | 0-255   |
  * PENDING     | TELEMETRY_VOLT    | 0x41 | C1_uint16________ | C2_uint16________ |         |         |         |         |         | VOLTS          | 0-1023  | 
  * PENDING     | TELEMETRY_CURRENT | 0x42 | C1_uint10________ | C2_uint10________ |         |         |         |         |         | AMPS           | 0-1023  |
  * PENDING     | TELEMETRY_RPM     | 0x43 | C1_uint16________ | C2_uint16________ |         |         |         |         |         | N/A            | 0-65535 |
  * PENDING     | TELEMETRY_ALT     | 0x44 |         |         |         |         |         |         |         |         |         | METERS         |         |
  * PENDING     | TELEMETRY_GPS     | 0x45 | STATE   | LATITUDE_int32_______________________ | LONGITUDE_int32______________________ | N/A            |         |
  * PENDING     | TELEMETRY_GPS2    | 0x46 |         |         |         |         |         |         |         |         |         |                |         |
  * PENDING     | TELEMETRY_HEADING | 0x47 |         |         |         |         |         |         |         |         |         | DEGREES        |         |
  * PENDING     | TELEMETRY_ACCL    | 0x48 |         |         |         |         |         |         |         |         |         | G              |         |
  *
  */

//###### TELEMETRY PROTOCOL #######
//	1 - OpenLRS NG telemetry protocol
//	2 - FRSKY compatible telemetry protocol
#define TELEMETRY_PROTOCOL 2

//####################
//### CODE SECTION ###
//####################

#define TELEMETRY_BUFFERSZ		0x21 // 33 bytes

#define TELEMETRY_RSSI			0x40
#define TELEMETRY_RSSI_PKTSZ	0x02
#define TELEMETRY_RSSI_ENABLED	0x01

#include "cb.h"
#include "telemetry_type.h"
#include "telemetry_sensor.h"

class telemetry_class {
	protected:
		cb tx_buffer;
		
		void tx_add( uint8_t* data, int bytes ) {										// Add n bytes of data to the transmit circular buffer
			if( bytes > this->tx_buffer.available() ) { return; }						// If buffer is full silently ignore the outgoing data
			for( uint8_t i = 0; i < bytes; i++ ) { this->tx_buffer.write( data[i] ); }	// Loop for n bytes and copy data to the transmit circular buffer
			free( data );																// Free the memory allocation for the data pointer
		}
		
		uint8_t size( uint8_t header ) {	// Based on the frame header get the packet size
			switch( header ) {
				case TELEMETRY_RSSI:
					return TELEMETRY_RSSI_PKTSZ;
					break;
			} return 0;
		}
		
	public:
		telemetry_sensor_rssi rssi;
		
		telemetry_class( void ) { // Class constructor
			this->tx_buffer.init( TELEMETRY_BUFFERSZ );
		}
		
		void transmit( void ) {
			if( this->tx_buffer.empty() ) { return; }   // Sneaky sanity check
			
			uint8_t size = this->size( this->tx_buffer.peek(0) );   // Reads the first byte without removing it from buffer
			if( size == 0 ) { return; }                             // Another sneaky sanity check
			
			uint8_t* data = (uint8_t *) malloc( sizeof(uint8_t) * size );                   // Packet size doesn't take into accoun the 1 byte header
			for( uint8_t i = 0; i <= size; i++ ) { data[ i ] = this->tx_buffer.read(); }    // We get around that by using the <= size and starting at zero
			
			#ifdef DEBUG
			Serial.print(" >> ");
			for( uint8_t i = 0; i <= size; i++ ) {
				Serial.print( "0x" );
				Serial.print( data[i], HEX);
				if( i < size ) { Serial.print( ", " ); }
			} Serial.println();
			#endif
			
			tx_packet( data, size );    // Send it to RX
			free( data );               // Deallocate pointer's memory, very important :-)
		}
		
		void queue( uint8_t type ) {
			if( TELEMETRY_RSSI_ENABLED ) {                                                  // We make sure that RSSI telemetry is enabled
				if( this->rssi.updated() ) {                                                // updated() will *only* return true once
					this->tx_add( this->rssi.exportPacket(), (TELEMETRY_RSSI_PKTSZ +1) );   // exportPacket() return a pointer but we do not care
				}                                                                           // about deallocation because tx_add() will do it for us
			}
		}
		
		void receive( void ) {
			uint8_t header = spiReadData();                                 // Fetch frame header and store it
			uint8_t   size = this->size( header );                          // Get the total packet size based on header
			uint8_t*  data = (uint8_t *) malloc( sizeof(uint8_t) * size );  // Allocate the required amount of memory
			
			data[0] = header;                                                   // Use the fetched header
			for( uint8_t i = 1; i <= size; i++ ) { data[i] = spiReadData(); }   // Fetch data and populate the frame
			
			#ifdef DEBUG
			Serial.print( "<< " );
			for( int8_t i = 0; i <= size; i++ ) {
				Serial.print( "0x" );
				Serial.print( data[i], HEX );
				if( i < size ) { Serial.print( ", " ); }
			} Serial.println();
			#endif
			
			switch( data[0] ) {                         // Process received frame
				case TELEMETRY_RSSI:                    // Checks if it is a RSS frame
					this->rssi.importPacket( data );    // Calls the packet importer for RSSI
					break;
				
			}
			
			free( data );   // Deallocate pointer's memory, very important :-)
		}
		
		virtual void forward( void ) {
			// Placeholder
		}
};

#if( TELEMETRY_PROTOCOL == 2 )
#include "telemetry_frsky.h"
#endif