all: astar_test tangent_test

DTYPESDIR=lib/dtypes
DTYPESSRC=$(DTYPESDIR)/src/darray.c $(DTYPESDIR)/src/dlist.c $(DTYPESDIR)/src/graph.c $(DTYPESDIR)/src/hset.c $(DTYPESDIR)/src/prqueue.c 
ALGOSRC=src/astar.c src/tangents.c src/vgraph.c

CC=gcc
CFLAGS=-std=c99 -lm
TESTFLAGS=-O0 -ggdb -Wall -Wextra
PERFFLAGS=-O3 -march=native

MEMTEST=valgrind --tool=memcheck --leak-check=full

astar_test:
	$(CC) $(CFLAGS) $(TESTFLAGS) $(DTYPESSRC) $(ALGOSRC) src/test/astar_test.c -I. -I$(DTYPESDIR) -o bin/astar_test
	$(MEMTEST) bin/astar_test

tangent_test:
	clang -std=c11 -O0 -ggdb -o bin/tangent_test -Iinclude -Ilib/dtypes/include src/test/tangent_test.c src/tangents.c lib/dtypes/src/darray.c lib/dtypes/src/graph.c src/astar.c lib/dtypes/src/hset.c  lib/dtypes/src/prqueue.c -lm
	valgrind --tool=memcheck --leak-check=full bin/tangent_test

vgraph_test:
	$(CC) $(CFLAGS) $(TESTFLAGS) $(DTYPESSRC) $(ALGOSRC) src/test/vgraph_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_test

vgraph_perf_test:
	$(CC) $(CFLAGS) $(PERFFLAGS) $(DTYPESSRC) $(ALGOSRC) src/test/vgraph_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_test
	bin/vgraph_test
