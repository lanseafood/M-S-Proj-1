# Authors: Mingyue Ma, Eisha Nathan, Stefan Henneking
# Date	 : 01/12/2015

# Georgia Institute of Technology
# CSE 6730: Road Traffic Simulation

CC       = gcc
CFLAGS   = -std=c99 #-Wall #-pedantic #-Werror
OPTFLAGS =
LIBS     = -lm
SOURCES  = event.c vehicle.c intersection.c section.c priorityQueue.c linkedList.c simEngine.c simApplication.c main.c
HEADERS  = event.h vehicle.h intersection.h section.h priorityQueue.h linkedList.h simEngine.h simApplication.h
OBJECTS  = $(SOURCES:.c=.o)

.PHONY: clean

# Define targets
TARGETS  = main
.PHONY: clean

# Make targets
$(TARGETS): $(OBJECTS)
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ $^ $(LIBS)

# Link objects
%.o: %.c
	$(CC) $(CFLAGS) $(OPTFLAGS) -o $@ -c $<

clean:
		rm -f $(OBJECTS)
		rm -f $(TARGETS)
		
# eof
