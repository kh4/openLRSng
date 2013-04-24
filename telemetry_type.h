 /*
  * openLRSng sensor type related classes
  * 
  */
  
class telemetry_type {
	protected:
		bool flag_changed;  // Flag indicating that the value was changed
		uint32_t timestamp; // Timestamp at when the value was changed
		
	public:
		bool updated( void ) {
			if( ! this->flag_changed ) { return false; } else {
				// As sensors are checked in sequence we can end up in always sending the
				// same sensor's data, this timestamp handicap could reduce the risk.
				if( (millis() - this->timestamp) > 100 ) {
					this->flag_changed = false; return true;
				} else { return false; }
			}
		}
};

class telemetry_type_uint8: public telemetry_type {
	protected:
		uint8_t cv, lv; // current value, last value
	
	public:
		long get( void       ) { return this->cv; }
		void set( uint8_t nv ) {
			if( this->cv == nv ) { return; }    // Only update the internal vars if the value changed
			this->lv = this->cv;                // Move current value to last value
			this->cv = nv;                      // Update current value
			this->flag_changed = true;          // Raise the changed flag
			this->timestamp = millis();         // Update the timestamp
		}
};

class telemetry_type_int32: public telemetry_type {
	protected:
		long cv, lv; // current value, last value
	
	public:
		long get( void    ) { return this->cv; }
		void set( long nv ) {
			if( this->cv == nv ) { return; }    // Only update the internal vars if the value changed
			this->lv = this->cv;                // Move current value to last value
			this->cv = nv;                      // Update current value
			this->flag_changed = true;          // Raise the changed flag
			this->timestamp = millis();         // Update the timestamp
		}
};