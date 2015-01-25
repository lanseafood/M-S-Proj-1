//
//  linkedList.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/13/2015
//

#include <stdio.h>
#include <stdlib.h>

#include "linkedList.h"

struct LinkedListElementType {
	ptr element;
	LinkedListElement next;
};

/**
 *	Implementation of struct PriorityQueueType for PriorityQueue
 *	-> implemented as a binary heap
 **/
struct LinkedListType {
	int counter;
	LinkedListElement head;
	LinkedListElement tail;
};

LinkedList create_list() {
	// Allocate memory for a new SimpleList
	LinkedList L = (LinkedList) malloc( sizeof( struct LinkedListType ) );
	if( L == NULL ) { fprintf(stderr,"ERROR from LL create_list(): malloc() failed\n"); exit(1); }
	
	// Set element counter to 0
	L->counter = 0;
	L->head = NULL;
	L->tail = NULL;
	
	return L;
}

void free_list( LinkedList L ) {
	if( L == NULL ) { fprintf(stderr,"Warning from LL free_list(): L is NULL\n"); return; }
	free( L );
	L = NULL;
}

int add_to_list( LinkedList L, ptr P ) {
    if( L == NULL ) { fprintf(stderr,"Warning from LL free_list(): L is NULL\n"); return -1; }
	if( P == NULL ) { fprintf(stderr,"Warning from LL free_list(): E is NULL\n"); return -1; }
	
	LinkedListElement E = malloc( sizeof( LinkedListElement ));
	if( E == NULL ) {
		fprintf(stderr,"ERROR from LL add_to_list(): malloc() failed\n"); exit(1);
	}
	E->element = P;
	E->next = NULL;
	if( L->head == NULL ) { L->head = E; /*printf("added head\n");*/ }
	if( L->tail != NULL ) {L->tail->next = E; /*printf("tail not null\n");*/}
	L->tail = E;
	L->counter++;
    
    return 0;
}

ptr peek_from_list( LinkedList L ) {
	if( L == NULL ) { fprintf(stderr,"Warning from LL peek_from_list(): L is NULL\n"); return NULL; }
	

	return L->head->element;
}

ptr poll_from_list( LinkedList L ) {
	if( L == NULL ) { fprintf(stderr,"Warning from LL remove_from_list(): L is NULL\n"); return NULL; }
	if( L->counter < 1 ) { fprintf(stderr,"Warning from LL remove_from_list(): L is empty\n"); return NULL; }
	ptr P = L->head->element;
	LinkedListElement E = L->head;
	L->head = L->head->next;
	free( E );
	L->counter--;
	if( L->counter < 1 ) L->tail = NULL;
	return P;
}

int get_list_counter( LinkedList L ) {
	if( L == NULL ) { fprintf(stderr,"Warning from LL get_list_counter(): L is NULL\n"); return -1; }
	return L->counter;
}

/* eof */
