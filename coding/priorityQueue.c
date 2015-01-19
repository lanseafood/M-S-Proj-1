//
//  priorityQueue.c
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "priorityQueue.h"

typedef uint_fast8_t  UINT8;
typedef uint_fast32_t UINT32;

/**
 *	Implementation of struct PriorityQueueType for PriorityQueue
 *	-> implemented as a binary heap
 **/
struct PriorityQueueType {
	queueElement *elements;
	UINT32 counter;
};

PriorityQueue create_priority_queue() {
	// Allocate memory for a new PriorityQueue
	PriorityQueue Q = (PriorityQueue) malloc( sizeof( struct PriorityQueueType ) );
	if( Q == NULL ) {
		fprintf(stderr,"ERROR from PQ create_priority_queue(): malloc() failed\n"); exit(1);
	}
	// Allocate memory for swapping element( elements[0] ) of the PriorityQueue
	Q->elements = malloc( sizeof( queueElement ) );
	if( Q->elements == NULL ) {
		fprintf(stderr,"ERROR from PQ create_priority_queue(): malloc() failed\n"); exit(1);
	}
	// Set element counter to 0
	Q->counter = 0;
	return Q;
}

void free_priority_queue( PriorityQueue Q ) {
	if( Q == NULL ) { fprintf(stderr,"Warning from PQ free_priority_queue(): Q is NULL\n"); return; }
	if( Q->elements != NULL ) {
		free( Q->elements );
		Q->elements = NULL;
	}
	free( Q );
	Q = NULL;
}

/**
 * Checks if element has parent (otherwise it is the head element)
 * Internal function
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @return		: 1 if it has parent, 0 otherwise
 **/
static inline UINT8 has_parent( PriorityQueue Q, UINT32 index ) {
	return ( index > 1) ? 1 : 0;
}

/**
 * Checks if element has left child
 * Internal function
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @return		: 1 if it has left child, 0 otherwise
 **/
static inline UINT8 has_left_child( PriorityQueue Q, UINT32 index ) {
	return ( index < 1 || 2*index > Q->counter ) ? 0 : 1;
}

/**
 * Checks if element has right child
 * Internal function
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @return		: 1 if it has right child, 0 otherwise
 **/
static inline UINT8 has_right_child( PriorityQueue Q, UINT32 index ) {
	return ( index < 1 || 2*index+1 > Q->counter ) ? 0 : 1;
}

/**
 * Get parent element
 * Internal function: No need to call "has_parent"
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @return		: Parent element
 **/
static inline queueElement get_parent( PriorityQueue Q, UINT32 index ) {
	return Q->elements[index/2];
}

/**
 * Get child element (left)
 * Internal function: No need to call "has_left_child"
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @return		: Left child element
 **/
static inline queueElement get_left_child( PriorityQueue Q, UINT32 index ) {
	return Q->elements[2*index];
}

/**
 * Get child element (right)
 * Internal function: No need to call "has_right_child"
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @return		: Right child element
 **/
static inline queueElement get_right_child( PriorityQueue Q, UINT32 index ) {
	return Q->elements[2*index+1];
}

/**
 * Set parent element
 * Internal function: No need to call "has_parent"
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @params E	: Element to be set as parent
 * return		: Index of parent
 **/
static inline UINT32 set_parent( PriorityQueue Q, UINT32 index, queueElement E ) {
	Q->elements[index/2] = E;
	return index/2;
}

/**
 * Set child element (left)
 * Internal function: No need to call "has_left_child"
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @params E	: Element to be set as left child
 * return		: Index of right child
 **/
static inline UINT32 set_left_child( PriorityQueue Q, UINT32 index, queueElement E ) {
	Q->elements[2*index] = E;
	return 2*index;
}

/**
 * Set child element (right)
 * Internal function: No need to call "has_right_child"
 *
 * @params Q	: Pointer to PriorityQueue
 * @params index: Index of initial element
 * @params E	: Element to be set as right child
 * return		: Index of right child
 **/
static inline UINT32 set_right_child( PriorityQueue Q, UINT32 index, queueElement E ) {
	Q->elements[2*index+1] = E;
	return 2*index+1;
}

int add( PriorityQueue Q, queueElement E ) {
	if( Q			== NULL ) { fprintf(stderr,"Warning from PQ add(): Q is NULL\n"); return -1; }
	if( Q->elements == NULL ) { fprintf(stderr,"Warning from PQ add(): Q->elements is NULL\n"); return -1; }
	if( E 			== NULL ) { fprintf(stderr,"Warning from PQ add(): E is NULL\n"); return -1; }
	
	// Reallocate with one QueueElement more
	Q->elements = realloc( Q->elements, sizeof( queueElement ) * ( Q->counter+2 ) );
	if( Q->elements == NULL ) {
		fprintf(stderr,"ERROR from PQ add(): realloc() failed\n"); exit(1);
	}
	Q->counter++;
	
	// Add element and preserve heap property
	Q->elements[Q->counter] = E;
	if( E == NULL ) {fprintf(stderr,"ERROR from PQ add(): E is NULL\n"); exit(1);}
	queueElement parent;
	int i = Q->counter;
	while( has_parent( Q, i ) ) {
		parent = get_parent( Q, i );
		if( parent == NULL ) {fprintf(stderr,"ERROR from PQ add(), parent is NULL\n"); exit(1);}
		// Swap Element E with parent, if E is smaller
		if( compare_to( E, parent ) ) {
			Q->elements[i] = parent;
			i = set_parent(Q, i, E);
			continue;
		} // Else: Q is min-Heap
		else { break; }
	}
	return 0;
}

int is_empty( PriorityQueue Q ) {
	if( Q == NULL ) { fprintf(stderr,"Warning from PQ is_empty(): Q is NULL\n"); return -1; }
	return ( Q->counter > 0 ) ? 0 : 1;
}

int get_count( PriorityQueue Q ) {
	if( Q == NULL ) { fprintf(stderr, "Warning from PQ get_count(): Q is NULL\n"); return -1; }
	return Q->counter;
}

queueElement peek( PriorityQueue Q ) {
	if( Q			== NULL ) { fprintf(stderr, "Warning from PQ peek(): Q is NULL\n"); return NULL; }
	if( is_empty( Q )		) { fprintf(stderr, "Warning from PQ peek(): Q is empty\n"); return NULL; }
	if( Q->elements == NULL ) { fprintf(stderr, "Warning from PQ peek(): Q->elements is NULL\n"); return NULL; }
	return Q->elements[1];
}

queueElement poll( PriorityQueue Q ) {
	if( Q			== NULL ) { fprintf(stderr, "Warning from PQ poll(): Q is NULL\n"); return NULL; }
	if( is_empty( Q )		) { fprintf(stderr, "Warning from PQ poll(): Q is empty\n"); return NULL; }
	if( Q->elements == NULL ) { fprintf(stderr, "Warning from PQ poll(): Q->elements is NULL\n"); return NULL; }
	
	// Remove head element ( store temporarily to elements[0] )
	// Swap last element of min-Heap to head position
	Q->elements[0] = Q->elements[1];
	Q->elements[1] = Q->elements[Q->counter];
	Q->counter--;
	
	// Preserve min-Heap
	UINT32 i = 1;
	queueElement leftChild, rightChild, E;
	while( has_left_child( Q, i ) ) {
		E = Q->elements[i];
		leftChild = get_left_child( Q, i );
		if( has_right_child( Q, i ) ) {
			// Get right child, if the Element E has one
			rightChild = get_right_child( Q, i );
			// Check if one of the children is smaller than E
			if( compare_to( leftChild, E ) || compare_to( rightChild, E ) ) {
				// If so, swap with smaller child
				if( compare_to( leftChild, rightChild ) ) {
					Q->elements[i] = leftChild;
					i = set_left_child( Q, i, E );
					continue;
				} else {
					Q->elements[i] = rightChild;
					i = set_right_child( Q, i, E );
					continue;
				} // Else: Q is min-Heap
			} else { break; }
		} // Else if: E has only left child
		else if( compare_to( leftChild, E ) ) {
			// Swap, if left child is smaller
			if( compare_to( leftChild, E ) ) {
				Q->elements[i] = leftChild;
				i = set_left_child( Q, i, E );
				continue;
			} // Else: Q is min-Heap
			else { break; }
		} // Else: Q is min-Heap
		else { break; }
	}
	
	// Reallocate with one QueueElement less
	Q->elements = realloc( Q->elements, sizeof( queueElement ) * ( Q->counter+1 ) );
	if( Q->elements == NULL ) {
		fprintf(stderr, "ERROR from PQ poll(): realloc() failed\n"); exit(1);
	}
	return Q->elements[0];
}

/* eof */
