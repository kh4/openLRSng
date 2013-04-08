/****************************************************
 * OpenLRSng telemetry generic code
 ****************************************************/

//###### TELEMETRY PROTOCOL #######
//	1 - OpenLRS NG telemetry protocol
//	2 - FRSKY compatible telemetry protocol
#define TELEMETRY_PROTOCOL 2

//###### TELEMETRY BUFFER SIZE #######
// You should only change this under _very_ special circumstances
#define TELEMETRY_MAX_BUFFER_SIZE 22


//####################
//### CODE SECTION ###
//####################

#include "cb.h"

class tCore {
	private:
		// nothing here
		
	protected:
		cb tx_buffer, rx_buffer;
		
		void tx_add( uint8_t* data, int bytes ) { // Add n bytes of data to the transmit circular buffer
			if( bytes > this->tx_buffer.available() ) { return; } // If buffer is full silently ignore the outgoing data
			for( uint8_t i = 0; i < bytes; i++ ) { this->tx_buffer.write( data[i] ); } // Loop for n bytes and copy data to the transmit circular buffer
		}
		
		void rx_add( uint8_t* data, int bytes ) { // Add n bytes of data to the receive circular buffer
			if( bytes > this->rx_buffer.available() ) { return; } // If buffer is full silently ignore the incoming data
			for( uint8_t i = 0; i < bytes; i++ ) { this->rx_buffer.write( data[i] ); } // Loop for n bytes and copy data to the receive circular buffer
		}
		
	public:
		tCore( void ) { // Class constructor
			this->tx_buffer.init( TELEMETRY_MAX_BUFFER_SIZE );
			this->rx_buffer.init( TELEMETRY_MAX_BUFFER_SIZE );
		}
		
		void send( void ) {
			//###### TELEMETRY TRANSMIT
			if( ! this->tx_buffer.empty() ) {
				uint8_t  c = 0;	// Byte counter
				uint8_t  byte;	// Temporary byte holder
				uint8_t* telemetry_packet = (uint8_t *) malloc( sizeof(uint8_t) ); // Complete packet
				
				while( ! this->tx_buffer.empty() ) { // Cycle until no more data is avaliable on the circular buffer
					c++;
					byte = this->tx_buffer.read();
					uint8_t* np = (uint8_t *) realloc( telemetry_packet, sizeof(uint8_t) * c ); // Increase memory allocation for c bytes
					if( np != NULL ) { telemetry_packet = np; } // On acllocatuion success overwrite the pointer
					telemetry_packet[ c - 1 ] = byte;
				}
/*
				Serial.print("send c:");
				Serial.println( c );
				for( uint8_t i = 0; i < c; i++ ) {
					Serial.print( ">> 0x" );
					Serial.println( telemetry_packet[i], HEX);
				}
*/
				if( c > 0 ) {
					// Transmit the packet and cleans allocated memory
					tx_packet( telemetry_packet, c );
					free( telemetry_packet );
				}
			}
		}
		
		void receive( void ) {
			// placeholder
		}
		
		virtual void queue( uint8_t* data, int bytes ) { // This function will be overwritten by each telemetry protocol,
			this->tx_add( data, bytes );			// it's now defined as a dummy high level for tx_add().
		}
};

#if( TELEMETRY_PROTOCOL == 2 )
#include "telemetry_frsky.h"
#elif
#include "telemetry_openlrs.h"
#endif