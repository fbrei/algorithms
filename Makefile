all: astar_test tangent_test

DTYPESDIR=lib/dtypes
DTYPESSRC=$(DTYPESDIR)/src/darray.c $(DTYPESDIR)/src/dlist.c $(DTYPESDIR)/src/graph.c $(DTYPESDIR)/src/hset.c $(DTYPESDIR)/src/prqueue.c 
ALGOSRC=src/astar.c src/tangents.c src/vgraph.c src/polygons.c

SFMTDIR=lib/sfmt
SFMTSRC=$(SFMTDIR)/SFMT.c

CC=gcc
override CFLAGS+=-std=c11 -I. -Ilib/dtypes -pipe -fstack-protector-strong -D_FORITFY_SOURCE=2 -Wall -Wextra -Werror=format-security -pedantic -fPIE -fpie
LD=$(CC)
LFLAGS=-lm -s -Wl,-z,now -Wl,-z,relro -pie -O1
TESTFLAGS=-Og -ggdb -Wall -Wextra -g
PERFFLAGS=-O3 -march=native

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
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/darray.c -o obj/darray.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/dlist.c -o obj/dlist.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/hset.c -o obj/hset.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/prqueue.c -o obj/prqueue.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/graph.c -o obj/graph.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/astar.c -o obj/astar.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/tangents.c -o obj/tangents.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/vgraph.c -o obj/vgraph.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/polygons.c -o obj/polygons.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/test/vgraph_total_test.c -o obj/vgraph_total_test.o $(CFLAGS)
	$(LD) obj/*.o -o bin/vgraph_total_test -lm
	rm -rf obj

line_sections_test:
	mkdir -p obj
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/darray.c -o obj/darray.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/dlist.c -o obj/dlist.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/hset.c -o obj/hset.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/prqueue.c -o obj/prqueue.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) lib/dtypes/src/graph.c -o obj/graph.o  $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/astar.c -o obj/astar.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/tangents.c -o obj/tangents.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/vgraph.c -o obj/vgraph.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/polygons.c -o obj/polygons.o $(CFLAGS)
	$(CC) -c $(TESTFLAGS) src/test/line_sections_test.c -o obj/line_sections_test.o $(CFLAGS)
	$(LD) obj/*.o -o bin/line_sections_test -lm
	rm -rf obj

measurement:
	mkdir -p obj
	$(CC) -c $(PERFFLAGS) lib/dtypes/src/darray.c -o obj/darray.o  $(CFLAGS)
	$(CC) -c $(PERFFLAGS) lib/dtypes/src/queue.c -o obj/queue.o  $(CFLAGS)
	$(CC) -c $(PERFFLAGS) lib/dtypes/src/dlist.c -o obj/dlist.o  $(CFLAGS)
	$(CC) -c $(PERFFLAGS) lib/dtypes/src/hset.c -o obj/hset.o  $(CFLAGS)
	$(CC) -c $(PERFFLAGS) lib/dtypes/src/prqueue.c -o obj/prqueue.o  $(CFLAGS)
	$(CC) -c $(PERFFLAGS) lib/dtypes/src/graph.c -o obj/graph.o  $(CFLAGS)
	$(CC) -c $(PERFFLAGS) src/astar.c -o obj/astar.o $(CFLAGS)
	$(CC) -c $(PERFFLAGS) src/tangents.c -o obj/tangents.o $(CFLAGS)
	$(CC) -c $(PERFFLAGS) src/vgraph.c -o obj/vgraph.o $(CFLAGS)
	$(CC) -c $(PERFFLAGS) src/polygons.c -o obj/polygons.o $(CFLAGS)
	$(CC) -c $(PERFFLAGS) src/test/vgraph_measurement.c -o obj/vgraph_measurement.o $(CFLAGS)
	$(CC) -c $(PERFFLAGS) lib/sfmt/SFMT.c -o obj/sfmt.o
	$(LD) obj/*.o -o bin/vgraph_measurement -lm
	rm -rf obj
