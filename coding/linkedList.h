//
//  linkedList.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/13/2015
//

#ifndef traffic_linkedList_h
#define traffic_linkedList_h

typedef void* ptr;
typedef struct LinkedListType *LinkedList;
typedef struct LinkedListElementType *LinkedListElement;

/**
 * Creates a new LinkedList
 *
 * @return: Pointer to LinkedList on success, exits otherwise
 **/
LinkedList create_list();

/**
 * Frees all space allocated by LinkedList
 *
 * @params L: Pointer to LinkedList
 **/
void free_list( LinkedList L );

/**
 * Adds a new element to the list
 *
 * @params L: Pointer to LinkedList
 * @params P: Pointer to Element to be added to the list
 * @return	: 0 on success, -1 otherwise
 **/
int add_to_list( LinkedList L, ptr P );

/**
 * Removes first element from list
 *
 * @params L: Pointer to LinkedList
 * @return  : Pointer to removed element on success, NULL otherwise
 **/
ptr remove_from_list( LinkedList L );

/**
 * Reads out the element counter of a list
 *
 * @params L: Pointer to LinkedList
 * @return	: #elements in LinkedList L on success, -1 otherwise
 **/
int get_list_counter( LinkedList L );

#endif

/* eof */
