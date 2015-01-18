//
//  intersection.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_intersection_h
#define traffic_intersection_h


typedef struct IntersectionType *Intersection;

typedef enum {
	GREEN,
	YELLOW,
	RED,
	INV
} Color;

typedef struct {
	int zoneID;
	double *times;
	Color init;
	Color light;
} Signal;


Intersection create_intersection();

int get_inter_zoneID(Intersection I);

int get_signal_zoneID(Signal *S);

int set_init(Signal *S, Color initial_color);

Color get_init(Signal *S);

int set_light(Signal *S, Color light);

Color get_light(Signal *S);

double get_ns_len(Intersection I);

double get_ew_len(Intersection I);

int add_car(Intersection I);

int get_num_cars(Intersection I);

#endif

/* eof */
