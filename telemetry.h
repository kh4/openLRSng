/****************************************************
 * OpenLRSng telemetry generic code
 ****************************************************/

//###### TELEMETRY PROTOCOL #######
//	1 - OpenLRS NG telemetry protocol
//	2 - FRSKY compatible telemetry protocol
#define TELEMETRY_PROTOCOL 1

//####################
//### CODE SECTION ###
//####################

#if( TELEMETRY_PROTOCOL == 1 )
#include "telemetry_openlrs.h"

#elif( TELEMETRY_PROTOCOL == 2 )
#include "telemetry_frsky.h"

#endif

void telemetry_send( void ) {

}