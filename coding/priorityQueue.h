//
//  priorityQueue.h
//  traffic
//
//  Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
//  01/12/2015
//

#ifndef traffic_priorityQueue_h
#define traffic_priorityQueue_h

typedef void* queueElement;
typedef struct PriorityQueueType *PriorityQueue;

// Compares two elements and specifies which one has higher priority
// Returns 1 if E1 has higher priority, and 0 if E2 has higher priority
// Function pointer has to be provided by user
extern int (*compare_to)( queueElement, queueElement );
/*extern int compare( queueElement E1, queueElement E2 );*/

/**
 * Creates a new PriorityQueue
 *
 * @return: Pointer to PriorityQueue
 **/
PriorityQueue create_priority_queue();

/**
 * Frees all space allocated by PriorityQueue
 *
 * @params Q: Pointer to PriorityQueue
 **/
void free_priority_queue( PriorityQueue Q );

/**
 * Adds element to the Queue
 *
 * @params Q	  : Pointer to PriorityQueue
 * @params element: Element to be added to the Queue
 * @return		  : 0 on success, -1 otherwise
 **/
int add( PriorityQueue Q, queueElement element );

/**
 * Check if Queue is empty or not
 *
 * @params Q: Pointer to PriorityQueue
 * @return	: 1 if Queue is empty, 0 otherwise
 **/
int is_empty( PriorityQueue Q );

/**
 * Gets the number of elements in the Queue
 *
 * @params Q: Pointer to PriorityQueue
 * @return	: #elements on success, -1 otherwise
 **/
int get_count(PriorityQueue Q);

/**
 * Read head of the queue
 *
 * @params Q: Pointer to PriorityQueue
 * @return	: Pointer to the head element, NULL if Queue is empy
 **/
queueElement peek( PriorityQueue Q );

/**
 * Remove head of the Queue
 *
 * @params Q: Pointer to PriorityQueue
 * @return	: Pointer to head element, NULL if the Queue is empty
 **/
queueElement poll( PriorityQueue Q );

#endif

/* eof */
