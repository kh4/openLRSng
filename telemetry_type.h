 /*
  * openLRSng sensor type related classes
  * 
  */

template <class T>
class telemetry_type {
	protected:
		T cv, lv;           // Current/Last Value
		bool flag_changed;  // Flag indicating that the value was changed
		uint32_t timestamp; // Timestamp at when the value was changed
		
	public:
		T get( void    ) { return this->cv; }
		
		void set( T nv ) {
			if( this->cv == nv ) { return; }    // Only update the internal vars if the value changed
			this->lv = this->cv;                // Move current value to last value
			this->cv = nv;                      // Update current value
			this->flag_changed = true;          // Raise the changed flag
			this->timestamp = millis();         // Update the timestamp
		}
		
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