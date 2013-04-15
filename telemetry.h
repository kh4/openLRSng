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
  * ------------ ---------------------- ------------------- ------ ------ --------- --------- --------- --------- --------- --------- --------- --------- --------- ---------------- ----------------------------------------------------
  * IN PROGRESS | Link quality          | TELEMETRY_RSSI    | 0x21 | 0x01 | RX_RSSI | RX_DROP |         |         |         |         |         |         |         | N/A            | 0-255         |
  * PENDING     | Voltmeter             | TELEMETRY_VOLT    | 0x21 | 0x02 | CHANNEL 1         | CHANNEL 2         |         |         |         |         |         | VOLTS          | 0-255         |
  * PENDING     | Amperemeter           | TELEMETRY_CURRENT | 0x21 | 0x03 | CHANNEL 1         | CHANNEL 2         |         |         |         |         |         | AMPS           | 0-255         |
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

#define CONFIG_FLAG				0xcf
#define TELEMETRY_FLAG			0x21
#define TELEMETRY_HEADER_SIZE	0x02

#define TELEMETRY_RSSI			0x01
//#define TELEMETRY_VOLT		0x02
//#define TELEMETRY_CURRENT		0x03
//#define TELEMETRY_RPM			0x04
//#define TELEMETRY_ALT			0x05
//#define TELEMETRY_GPS			0x06
//#define TELEMETRY_GPS_AS		0x07
//#define TELEMETRY_HEADING		0x08
//#define TELEMETRY_ACCL		0x09

#define TELEMETRY_BUFFER_SIZE	0x21 // 33 bytes

class sensor_db_class {
	protected:
		uint32_t timestamp;
		uint8_t packet_size;
		bool pending_transmit;

	public:
		uint8_t size( void ) { return this->packet_size; }
		void size( uint8_t val ) { this->packet_size = val; }
		void updated( bool val ) { this->pending_transmit = val; }
		
		bool updated( void ) {
			if( ! this->pending_transmit ) { return false; }
			else { this->pending_transmit = false; return true; }
			//TODO Should I have here some kind of time handicap ?
			// As sensors are checked in sequence we can end up in always sending the
			// same sensor's data; a timestamp handicap would reduce the risk.
		}
};

class rssi_class: public sensor_db_class {
	private:
		struct {
			uint8_t tx;
			uint8_t drop;
		} a, b;
		
	public:
		rssi_class( void ) { this->size( 4 ); }
		uint8_t tx( void ) { return this->a.tx; }
		uint8_t drop( void ) { return this->a.drop; }
		
		void tx( uint8_t value ) { if( value != this->a.tx ) {
				this->pending_transmit = true;
				this->timestamp = millis();
				this->b.tx = this->a.tx;
				this->a.tx = value;
		} }
		
		void drop( uint8_t value ) { if( value != this->a.drop ) {
				this->pending_transmit = true;
				this->timestamp = millis();
				this->b.drop = this->a.drop;
				this->a.drop = value;
		} }
		
		uint8_t* exportPacket( void ) {
			uint8_t* data = (uint8_t *) malloc( sizeof(uint8_t) * this->size() );
			data[0] = TELEMETRY_FLAG;
			data[1] = TELEMETRY_RSSI;
			data[2] = this->a.tx;
			data[3] = this->a.drop;
			return data;
		}
		
		void importPacket( uint8_t* data ) {
			if( data[1] != TELEMETRY_RSSI ) { return; } // Sanity check
			this->tx( data[2] );
			this->drop( data[3] );
		}
};

class telemetry_class {
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
		struct {
			rssi_class rssi;
			//voltmeter_class voltmeter;
			//ammeter_class ammeter;
		} db;
		
		telemetry_class( void ) { // Class constructor
			this->tx_buffer.init( TELEMETRY_BUFFER_SIZE );
			this->rx_buffer.init( TELEMETRY_BUFFER_SIZE );
		}
		
		void transmit( void ) {
		//TODO Only allow the transmition of ONE packet per cycle
			if( ! this->tx_buffer.empty() ) {
				uint8_t  c = 0;	// Byte counter
				uint8_t  byte;	// Temporary byte holder
				uint8_t* data = (uint8_t *) malloc( sizeof(uint8_t) ); // Complete packet
				
				while( ! this->tx_buffer.empty() ) {								// Cycle until no more data is avaliable on the buffer
					c++; byte = this->tx_buffer.read();								// Increase read bytes counter and get another byte from buffer
					uint8_t* np = (uint8_t *) realloc( data, sizeof(uint8_t) * c );	// Increase memory allocation
					if( np != NULL ) { data = np; }									// Overwrite the original pointer
					data[ c - 1 ] = byte;											// Stores the new byte
				}
				
				#ifdef DEBUG
				Serial.print(" >> ");
				for( uint8_t i = 0; i < c; i++ ) {
					Serial.print( "0x" );
					Serial.print( data[i], HEX);
					Serial.print( ", " );
				} Serial.println();
				#endif
				
				tx_packet( data, c );	// Send it over transparent serial link to RX
				free( data );
			}
		}
		
		void queue( uint8_t type ) {
			if( this->db.rssi.updated() ) {						// Attention updated() clears the flag at execution, so only returns turn once
				uint8_t* data = this->db.rssi.exportPacket();	// Fetch the complete packet
				this->tx_add( data, this->db.rssi.size() );		// Add the packet to tx queue
				free( data );									//WARNING the caller is responsable for freeing the memory
			}
		}
		
		void receive( void ) {
			if( this->rx_buffer.available() >= 11 ) {
				uint8_t* data = (uint8_t *) malloc( sizeof(uint8_t) * 11 );
				for( uint8_t i = 0; i < 11; i++ ) { data[i] = spiReadData(); }
				
				#ifdef DEBUG
				Serial.print( "<< " );
				for( int8_t i = 0; i < 11; i++ ) {
					Serial.print( "0x" );
					Serial.print( data[i], HEX );
					Serial.print( ", " );
				} Serial.println();
				#endif
				
				switch( data[1] ) {
					case TELEMETRY_RSSI:
						this->db.rssi.importPacket( data );
						break;
					
				}
				
				free( data );
			}
		}
		
		virtual void forward( void ) {
			// Placeholder
		}
};

#if( TELEMETRY_PROTOCOL == 2 )
#include "telemetry_frsky.h"
#endif