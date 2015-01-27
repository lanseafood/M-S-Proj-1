//
//  section.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_section_h
#define traffic_section_h

// Declare abstract type of Section
typedef struct SectionType *Section;

Section create_section(int zoneID, double vehicle_len, double safety_dist_queue);

int get_zoneID(Section S);

double get_len(Section S);

int change_north_vehicles(Section S, int val);
int change_south_vehicles(Section S, int val);

int get_north_vehicles(Section S);
int get_south_vehicles(Section S);

#endif

/* eof */
