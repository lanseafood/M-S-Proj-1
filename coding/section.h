//
//  section.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_section_h
#define traffic_section_h

// Declare abstract type of Vehicle
typedef struct SectionType *Section;


Section create_section(int zoneID);

int get_zoneID(Section S);

double get_len(Section S);

int add_north_cars(Section S);

int add_south_cars(Section S);

int get_north_cars(Section S);

int get_south_cars(Section S);

#endif

/* eof */
