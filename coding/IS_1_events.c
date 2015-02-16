//
//  IS_1_events.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

static int IS_1_choose_route_and_lane( Direction dir, int dest, Vehicle V );
static int IS_1_red_right_turn( Direction D );
static int IS_1_left_turn( Direction D );

//////////////////////////////////////////////////////////////////////////////////////////////
// Choose appropriate lane for vehicle
static int IS_1_choose_route_and_lane( Direction dir, int dest, Vehicle V ) {
	int laneID = 0;
	switch ( dir ) {
		case NORTH:
		{
			if( dest == 223 ) { set_route( V, RIGHT ); set_departDir( V, WEST ); laneID = 3; }
			else if( dest == 202 ) { set_route( V, LEFT ); set_departDir( V, EAST ); laneID = 1; }
			else if( dest == 201 ) { set_route( V, STRAIGHT ); set_departDir( V, SOUTH ); laneID =
				( get_list_counter( get_lane_queue( IS_1, NORTH, 3 ) ) <
				 get_list_counter( get_lane_queue( IS_1, NORTH, 2 ) ) ) ? 3 : 2; }
			else { fprintf(stderr,"Error from IS_1_choose_lane(): unexpected destination zone\n"); exit(1); }
			break;
		}
		case EAST:
		{
			if( dest == 223 ) { set_route( V, STRAIGHT ); set_departDir( V, WEST ); laneID =
				( get_list_counter( get_lane_queue( IS_1, EAST, 3 ) ) <
				 get_list_counter( get_lane_queue( IS_1, EAST, 2 ) ) ) ? 3 : 2; }
			else if( dest == 201 ) { set_route( V, LEFT ); set_departDir( V, SOUTH ); laneID = 1; }
			else { set_route( V, RIGHT ); set_departDir( V, NORTH ); laneID = 3; }
			break;
		}
		case SOUTH:
		{
			if( dest == 202 ) { set_route( V, RIGHT ); set_departDir( V, EAST ); laneID = 3; }
			else if( dest == 223 ) { set_route( V, LEFT ); set_departDir( V, WEST ); laneID = 1; }
			else { set_route( V, STRAIGHT ); set_departDir( V, NORTH ); laneID =
				( get_list_counter( get_lane_queue( IS_1, SOUTH, 3 ) ) <
				 get_list_counter( get_lane_queue( IS_1, SOUTH, 2 ) ) ) ? 3 : 2; }
			break;
		}
		case WEST:
		{
			if( dest == 202 ) { set_route( V, STRAIGHT ); set_departDir( V, EAST ); laneID =
				( get_list_counter( get_lane_queue( IS_1, WEST, 3 ) ) <
				 get_list_counter( get_lane_queue( IS_1, WEST, 2 ) ) ) ? 3 : 2; }
			else if( dest == 201 ) { set_route( V, RIGHT ); set_departDir( V, SOUTH ); laneID = 4; }
			else { set_route( V, LEFT ); set_departDir( V, NORTH ); laneID = 1; }
			break;
		}
		default:
			fprintf(stderr,"Error from IS_1_choose_route_and_lane(): unexpected vehicle direction\n"); exit(1);
	}
	return laneID;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Check if red right turn for direction D is possible based on IS 1 traffic
static int IS_1_red_right_turn( Direction D ) {
	int redRightTurn = 0;
	int rightTurnLane = get_numLanes( IS_1 )[D];
	if( get_list_counter( get_lane_queue( IS_1, D, rightTurnLane ) ) == 0 ) return 0;
	if( get_lane_flag( IS_1, D, rightTurnLane ) == 1 ) return 0;
	Event E = peek_from_list( get_lane_queue( IS_1, D, rightTurnLane ) );
	if( get_scheduled( E ) == 1 ) return 0;
	Vehicle V = get_object( E );
	if( get_route( V ) != RIGHT ) return 0;
	switch( D ) {
		case NORTH:
		{
			if(   get_lane_counter( IS_1, SOUTH, 1 ) == 0
			   && get_lane_counter( IS_1,  EAST, 2 ) == 0
			   && get_lane_counter( IS_1,  EAST, 3 ) == 0
			   ) redRightTurn = 1;
			break;
		}
		case  EAST:
		{
			if(   get_lane_counter( IS_1,  WEST, 1 ) == 0
			   && get_lane_counter( IS_1, SOUTH, 2 ) == 0
			   && get_lane_counter( IS_1, SOUTH, 3 ) == 0
			   ) redRightTurn = 1;
			break;
		}
		case SOUTH:
		{
			if(   get_lane_counter( IS_1, NORTH, 1 ) == 0
			   && get_lane_counter( IS_1,  WEST, 2 ) == 0
			   && get_lane_counter( IS_1,  WEST, 3 ) == 0
			   ) redRightTurn = 1;
			break;
		}
		case  WEST:
		{
			if(   get_lane_counter( IS_1,  EAST, 1 ) == 0
			   && get_lane_counter( IS_1, NORTH, 2 ) == 0
			   && get_lane_counter( IS_1, NORTH, 3 ) == 0
			   ) redRightTurn = 1;
			break;
		}
		default:
			fprintf(stderr,"Error from IS_1_red_right_turn(): invalid direction\n"); exit(1);
	}
	return redRightTurn;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Check if left turn for direction D is possible based on IS 1 traffic
static int IS_1_left_turn( Direction D ) {
	int leftTurn = 0;
	if( get_list_counter( get_lane_queue( IS_1, D, 1 ) ) == 0 ) return 0;
	
	Event E = peek_from_list( get_lane_queue( IS_1, D, 1 ) );
	if( get_scheduled( E ) == 1 ) return 0;
	Vehicle V = get_object( E );
	if( get_route( V ) != LEFT ) return 0;
	
	if( get_light( IS_1, D, 1 ) == RED ) return 0;
	if( get_lane_flag( IS_1, D, 1 ) == 1 ) return 0;
	if( get_protected( IS_1 ) == 1 ) return 1;
	
	switch( D ) {
		case NORTH:
		{
			if(   get_lane_counter( IS_1, SOUTH, 2 ) == 0
			   && get_lane_counter( IS_1, SOUTH, 3 ) == 0
			   ) leftTurn = 1;
			break;
		}
		case  EAST:
		{
			if(   get_lane_counter( IS_1, WEST, 2 ) == 0
			   && get_lane_counter( IS_1, WEST, 3 ) == 0
			   && get_lane_counter( IS_1, WEST, 4 ) == 0
			   ) leftTurn = 1;
			break;
		}
		case SOUTH:
		{
			if(   get_lane_counter( IS_1, NORTH, 2 ) == 0
			   && get_lane_counter( IS_1, NORTH, 3 ) == 0
			   ) leftTurn = 1;
			break;
		}
		case  WEST:
		{
			if(   get_lane_counter( IS_1, EAST, 2 ) == 0
			   && get_lane_counter( IS_1, EAST, 3 ) == 0
			   ) leftTurn = 1;
			break;
		}
		default:
			fprintf(stderr,"Error from IS_1_left_turn(): invalid direction\n"); exit(1);
	}
	return leftTurn;
}

//////////////////////////////////////////////////////////////////////////////////////////////
static void IS_1_arrival( void* P ) {
	Event E = (Event) P;
	check_and_print_vehicle_event( E );
	Vehicle V = get_object( E );
	Direction dir = get_dir( V );
	int dest = get_destination( V );
	
	set_event_type( E, IS_1_ENTERING );
	set_callback( E, IS_1_entering );
	
	// Queue up or enter intersection depending on vehicle destination
	int newLane = IS_1_choose_route_and_lane( dir, dest, V );
	
	set_laneID( V, newLane );
	add_to_list( get_lane_queue( IS_1, dir, newLane ), E );
	// Check traffic signal
	if( get_light( IS_1, dir, newLane ) == RED ) {
		// Put vehicle in queue
		set_velocity( V, 0.0 );
		if( newLane == get_numLanes( IS_1 )[dir] && IS_1_red_right_turn( dir ) ) {
			Event entering = peek_from_list( get_lane_queue( IS_1, dir, newLane ) );
			set_timestamp( entering, get_sim_time() );
			schedule_event( entering );
		}
	} else {
		// If queue is empty, and no vehicle is currently entering,
		// schedule entering event directly
		if(   get_list_counter( get_lane_queue( IS_1, dir, newLane ) ) == 1
		   && get_lane_flag( IS_1, dir, newLane ) == 0 )
		{
			// Don't enter if permitted left turn and there is traffic in opposite direction
			if( ! ( get_route( V ) == LEFT && IS_1_left_turn( dir ) == 0 ) ) {
				schedule_event( E );
			} else { set_velocity( V, 0.0 ); }
		}
		else { set_velocity( V, 0.0 ); }
	}
	set_wait_time_buf( V, get_sim_time() );
}

//////////////////////////////////////////////////////////////////////////////////////////////
static void IS_1_entering( void* P ) {
	Event E = (Event) P;
	check_and_print_vehicle_event( E );
	Vehicle V = get_object( E );
	Direction dir = get_dir( V );
	int laneID = get_laneID( V );
	
	if( get_route( V ) == LEFT && IS_1_left_turn( dir ) == 0 ) { set_velocity( V, 0.0 ); return; }
	if( get_departDir( V ) == NORTH && get_congestion_flag( S_2, NORTH ) ) { set_velocity( V, 0.0 ); return; }
	// Check if this event is first in the queue
	if( peek_from_list( get_lane_queue( IS_1, dir, laneID ) ) != E ) {
		fprintf(stderr,"Error from IS_1_entering(): E is not first in queue\n"); exit(1);
	}
	// Check if no other vehicle is currently entering
	// Check also congestion here
	if( get_lane_flag( IS_1, dir, laneID ) == 0 ) {
		set_event_type( E, IS_1_CROSSING );
		set_callback( E, IS_1_crossing );
		set_timestamp( E, get_sim_time()+SDT );
		// Pre-calculate temp distance of V such that crossing knows how far V has traveled
		set_temp_distance( V, calc_distance( get_velocity(V), SDT ) );
		if( get_velocity(V) < VEL ) {
			set_velocity(V, calc_velocity( get_velocity(V), SDT ) );
		}
		set_lane_flag( IS_1, dir, laneID, 1 );
		change_lane_counter( IS_1, dir, laneID, 1 );
		schedule_event( poll_from_list( get_lane_queue( IS_1, dir, laneID ) ) );
		add_wait_time( V, get_sim_time()-get_wait_time_buf( V ) );
		set_wait_time_buf( V, 0.0 );
		// Decrement section vehicle counter
		if( dir == NORTH ) change_south_vehicles( S_2, -1 );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
static void IS_1_crossing( void* P ) {
	Event E = (Event) P;
	check_and_print_vehicle_event( E );
	Vehicle V = get_object( E );
	Direction dir = get_dir( V );
	Route route = get_route( V );
	
	set_event_type( E, IS_1_DEPARTURE );
	set_callback( E, IS_1_departure );
	
	// Allow next vehicle to enter this lane
	int laneID = get_laneID( V );
	set_lane_flag( IS_1, dir, laneID, 0 );
	
	// Time to departure is computed by using current position
	// (temp distance that was precalculated in entering event handler)
	double crossingDistance = get_crossing_distance( IS_1, dir, route );
	double distance_to_departure = fmax( 0.0, crossingDistance - get_temp_distance(V) );
	double time_to_departure = calc_time( get_velocity(V), distance_to_departure );
	if( get_velocity(V) < VEL ) {
		set_velocity(V, calc_velocity( get_velocity(V), time_to_departure) );
	}
	set_timestamp( E, get_sim_time() + time_to_departure );
	// Schedule departure
	schedule_event( E );
	
	// Schedule entering event for following vehicle, if signal is still yellow/green
	if( get_list_counter( get_lane_queue( IS_1, dir, laneID ) ) > 0 ) {
		Event nextEvent = peek_from_list( get_lane_queue( IS_1, dir, laneID ) );
		Vehicle nextVehicle = get_object( nextEvent );
		if( get_light( IS_1, dir, laneID ) != RED ) {
			// Don't enter if permitted left turn and there is traffic in opposite direction
			if( ! ( get_route( nextVehicle ) == LEFT && IS_1_left_turn( dir ) == 0 ) ) {
				if( !(get_scheduled( nextEvent )) ) {
					set_timestamp( nextEvent, get_sim_time() );
					schedule_event( nextEvent );
				}
			}
		} else if( laneID == get_numLanes( IS_1 )[dir] && IS_1_red_right_turn( dir ) ) {
			set_timestamp( nextEvent, get_sim_time() );
			schedule_event( nextEvent );
		} else ;
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////
static void IS_1_departure( void* P ) {
	Event E = (Event) P;
	check_and_print_vehicle_event( E );
	Vehicle V = get_object( E );
	
	Direction dir = get_dir( V );
	Direction departDir = get_departDir( V );
	
	set_temp_distance( V, 0.0 );
	
	// Remove vehicle from intersection counter
	change_lane_counter( IS_1, dir, get_laneID(V), -1 );
	
	// Check if vehicle has blocked permitted LEFT turn vehicle from opposite direction
	// ( could only have happened if vehicle didn't make a left turn )
	if( get_route( V ) != LEFT ) {
		Direction opposite = (dir+2)%4;
		if( IS_1_left_turn( opposite ) == 1 ) {
			Event entering = peek_from_list( get_lane_queue( IS_1, opposite, 1 ) );
			set_timestamp( entering, get_sim_time() );
			schedule_event( entering );
		}
	}
	
	// Vehicle may have blocked a red right turn from direction (departDir+1)%4
	// e.g. NORTH departure may have blocked EAST red right turn
	Direction redTurn = (departDir+1)%4;
	if( IS_1_red_right_turn( redTurn ) ) {
		Event rightTurn = peek_from_list( get_lane_queue( IS_1, redTurn, get_numLanes( IS_1 )[redTurn] ) );
		set_timestamp( rightTurn, get_sim_time() );
		schedule_event( rightTurn );
	}
	
	// Schedule next event
	if( departDir == NORTH ) {
		set_event_type( E, IS_2_ARRIVAL );
		set_callback  ( E, IS_2_arrival );
		double leng = get_len( S_2 );
		double timeToArrival = calc_time( get_velocity( V ), get_len( S_2 ) );
		set_timestamp ( E, get_sim_time() + timeToArrival );
		set_velocity( V, calc_velocity( get_velocity( V ), timeToArrival ) );
		// Increment section vehicle counter
		change_north_vehicles( S_2, 1 );
		set_dir( V, SOUTH );
		
	} else {
		set_event_type( E, GLOBAL_DEPARTURE );
		set_callback  ( E, global_departure );
	}
	schedule_event( E );
}


/* eof */
