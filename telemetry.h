 /*
  * OpenLRS NG GENERIC TELEMETRY CODE
  * 
  * //###### CORE PROTOCOL #######
  * 0x21 - Telemetry frame
  * 0x5e - Channel values
  * 0xcf - Configuration frame
  * 0xf5 - Failsafe values
  * 
  * //###### TELEMETRY PACKET #######
  * [0x21][FLAG][DATA1][DATA2][DATA3][DATA4][DATA5][DATA6][DATA7][DATA8][DATA9]
  *
  * WARNING: The following table is subject to change and it's current purpose is to act as a early draft.
  * 
  * STATUS      | SENSOR TYPE           | DEFINE            | TF   | FLAG | DATA1   | DATA2   | DATA3   | DATA4   | DATA5   | DATA6   | DATA7   | DATA8   | DATA9   | UNIT           | RANGE         | NOTES
  * ------------ ---------------------- ------------------- ------ ------ --------- --------- --------- --------- --------- --------- --------- --------- --------- ---------------- --------------- -------------------------------------
  * IN PROGRESS | Link quality          | TELEMETRY_RSSI    | 0x21 | 0x01 | RX_RSSI | TX_RSSI | RX_DROP | TR_DROP |         |         |         |         |         | N/A            | 0-255         |
  * PENDING     | Voltmeter             | TELEMETRY_VOLT    | 0x21 | 0x02 | VOLT1   | VOLT2   | VOLT3   | VOLT4   |         |         |         |         |         | VOLTS          | 0-255         |
  * PENDING     | Amperemeter           | TELEMETRY_CURRENT | 0x21 | 0x03 | AMP1    | AMP2    | AMP3    | AMP 4   |         |         |         |         |         | AMPS           | 0-255         |
  * PENDING     | RPM sensor            | TELEMETRY_RPM     | 0x21 | 0x04 | RPM SENSOR 1      | RPM SENSOR 2      |         |         |         |         |         | N/A            | 0-65535       |
  * PENDING     | Altimeter             | TELEMETRY_ALT     | 0x21 | 0x05 | VALUE1 ---------- | VALUE2  |         |         |         |         |         |         | METERS         |               | VALUE1.VALUE2
  * PENDING     | GPS Longitude         | TELEMETRY_GPS     | 0x21 | 0x06 | DEGREES | MINUTES | SECONDS | N/S     | DEGREES | MINUTES | SECONDS | E/W     |         | N/A            |               |
  * PENDING     | GPS Altitude & Speed  | TELEMETRY_GPS_AS  | 0x21 | 0x07 | ALT ------------- | SPEED   |         |         |         |         |         |         | METERS & KNOTS |               |
  * PENDING     | Heading               | TELEMETRY_HEADING | 0x21 | 0x08 | VALUE1 ---------- | VALUE2  |         |         |         |         |         |         | DEGREES        |               | VALUE1.VALUE2
  * PENDING     | Accelerometer         | TELEMETRY_ACCL    | 0x21 | 0x09 | X --------------------------| Y --------------------------| Z --------------------------| G              |               |
  * 
  */
 
//###### TELEMETRY PROTOCOL #######
//	1 - OpenLRS NG telemetry protocol
//	2 - FRSKY compatible telemetry protocol
#define TELEMETRY_PROTOCOL 2

//####################
//### CODE SECTION ###
//####################

#include "cb.h"
//#include <util/atomic.h>

#define TELEMETRY_FLAG		0x21

#define TELEMETRY_RSSI		0x01
#define TELEMETRY_VOLT		0x02
#define TELEMETRY_CURRENT	0x03
#define TELEMETRY_RPM		0x04
#define TELEMETRY_ALT		0x05
#define TELEMETRY_GPS		0x06
#define TELEMETRY_GPS_AS	0x07
#define TELEMETRY_HEADING	0x08
#define TELEMETRY_ACCL		0x09

#define TELEMETRY_MAX_BUFFER_SIZE 11

struct telemetry_rssi {
	uint8_t rx_rssi, tx_rssi, rx_drop, tx_drop;
};

struct telemetry_volt {
	uint8_t volt1, volt2, volt3, volt4;
};

unsigned long telemetrySendTimer = 0;
unsigned long telemetrySendTimestamp = 0;

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
		
		void flush( void ) {
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
#ifdef DEBUG
				Serial.print(" >> ");
				for( uint8_t i = 0; i < c; i++ ) {
					Serial.print( "0x" );
					Serial.print( telemetry_packet[i], HEX);
					Serial.print( ", " );
				} Serial.println();
#endif
				if( c > 0 ) {
					// Transmit the packet and cleans allocated memory
					tx_packet( telemetry_packet, c );
					free( telemetry_packet );
				} else { Serial.println( "Refused to send an empty telemetry packet." ); }
			}
		}
		
		void receive( void ) {
			// placeholder
		}
		
		void queue( telemetry_rssi packet ) {
			uint8_t* data = (uint8_t *) malloc( 6 );
			
			data[ 0 ] = TELEMETRY_FLAG;
			data[ 1 ] = TELEMETRY_RSSI;
			data[ 2 ] = packet.rx_rssi;
			data[ 3 ] = packet.tx_rssi;
			data[ 4 ] = packet.rx_drop;
			data[ 5 ] = packet.tx_drop;
			
			this->tx_add( data, 6 );
			free( data );
		}
};

#if( TELEMETRY_PROTOCOL == 2 )
#include "telemetry_frsky.h"
#elif
#include "telemetry_openlrs.h"
#endif