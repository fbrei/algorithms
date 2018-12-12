all: astar_test tangent_test

DTYPESDIR=lib/dtypes
DTYPESSRC=$(DTYPESDIR)/src/darray.c $(DTYPESDIR)/src/dlist.c $(DTYPESDIR)/src/graph.c $(DTYPESDIR)/src/hset.c $(DTYPESDIR)/src/prqueue.c 
ALGOSRC=src/astar.c src/tangents.c src/vgraph.c src/polygons.c

SFMTDIR=lib/sfmt
SFMTSRC=$(SFMTDIR)/SFMT.c

CC=gcc
CFLAGS=-std=c99 -lm -I. -Ilib/dtypes -pipe
TESTFLAGS=-Og -ggdb -Wall -Wextra -g
PERFFLAGS=-O3 -march=native -s 

MEMTEST=valgrind --tool=memcheck --leak-check=full

astar_test:
	$(CC) $(CFLAGS) $(TESTFLAGS) $(DTYPESSRC) $(ALGOSRC) src/test/astar_test.c -I. -I$(DTYPESDIR) -o bin/astar_test
	$(MEMTEST) bin/astar_test

tangent_test:
	clang -std=c11 -O0 -ggdb -o bin/tangent_test -Iinclude -Ilib/dtypes/include src/test/tangent_test.c src/tangents.c lib/dtypes/src/darray.c lib/dtypes/src/graph.c src/astar.c lib/dtypes/src/hset.c  lib/dtypes/src/prqueue.c -lm
	valgrind --tool=memcheck --leak-check=full bin/tangent_test

vgraph_test:
	$(CC) $(CFLAGS) $(TESTFLAGS) $(DTYPESSRC) $(ALGOSRC) $(SFMTSRC) src/test/vgraph_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_test
	$(MEMTEST) bin/vgraph_test

vgraph_perf_test:
	$(CC) $(CFLAGS) $(PERFFLAGS) $(DTYPESSRC) $(ALGOSRC) $(SFMTSRC) src/test/vgraph_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_test
	bin/vgraph_test

vgraph_single_test:
	$(CC) $(CFLAGS) $(PERFFLAGS) $(DTYPESSRC) $(ALGOSRC) $(SFMTSRC) src/test/vgraph_single_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_single_test

vgraph_polygon_test:
	$(CC) $(CFLAGS) $(TESTFLAGS) $(DTYPESSRC) $(ALGOSRC) $(SFMTSRC) src/test/vgraph_polygons_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_polygon_test

vgraph_total_test:
	mkdir -p obj
	$(CC) -c $(CFLAGS) $(PERFFLAGS) lib/dtypes/src/darray.c -o obj/darray.o 
	$(CC) -c $(CFLAGS) $(PERFFLAGS) lib/dtypes/src/dlist.c -o obj/dlist.o 
	$(CC) -c $(CFLAGS) $(PERFFLAGS) lib/dtypes/src/hset.c -o obj/hset.o 
	$(CC) -c $(CFLAGS) $(PERFFLAGS) lib/dtypes/src/prqueue.c -o obj/prqueue.o 
	$(CC) -c $(CFLAGS) $(PERFFLAGS) lib/dtypes/src/graph.c -o obj/graph.o 
	$(CC) -c $(CFLAGS) $(PERFFLAGS) src/astar.c -o obj/astar.o
	$(CC) -c $(CFLAGS) $(PERFFLAGS) src/tangents.c -o obj/tangents.o
	$(CC) -c $(CFLAGS) $(PERFFLAGS) src/vgraph.c -o obj/vgraph.o
	$(CC) -c $(CFLAGS) $(PERFFLAGS) src/polygons.c -o obj/polygons.o
	$(CC) -c $(CFLAGS) $(PERFFLAGS) src/test/vgraph_total_test.c -o obj/vgraph_total_test.o
	$(CC) $(CFLAGS) obj/*.o -o bin/vgraph_total_test
	rm -rf obj
