all: astar_test tangent_test

DTYPESDIR=lib/dtypes
DTYPESSRC=$(DTYPESDIR)/src/darray.c $(DTYPESDIR)/src/dlist.c $(DTYPESDIR)/src/graph.c $(DTYPESDIR)/src/hset.c $(DTYPESDIR)/src/prqueue.c 
ALGOSRC=src/astar.c src/tangents.c src/vgraph.c

CC=clang
CFLAGS=-std=c11 -lm
TESTFLAGS=-O0 -ggdb -Wall -Wextra
PERFFLAGS=-O3 -pg -march=native

MEMTEST=valgrind --tool=memcheck --leak-check=full

astar_test:
	clang -std=c11 -O0 -ggdb -o bin/astar_test -Iinclude -Ilib/dtypes/include src/astar.c src/test/astar_test.c lib/dtypes/src/darray.c lib/dtypes/src/prqueue.c lib/dtypes/src/graph.c lib/dtypes/src/hset.c -lm 
	valgrind --tool=memcheck --leak-check=full bin/astar_test

tangent_test:
	clang -std=c11 -O0 -ggdb -o bin/tangent_test -Iinclude -Ilib/dtypes/include src/test/tangent_test.c src/tangents.c lib/dtypes/src/darray.c lib/dtypes/src/graph.c src/astar.c lib/dtypes/src/hset.c  lib/dtypes/src/prqueue.c -lm
	valgrind --tool=memcheck --leak-check=full bin/tangent_test

perf_test:
	clang -std=c11 -O3 -o bin/tangent_test -Iinclude -Ilib/dtypes/include src/test/tangent_test.c src/tangents.c lib/dtypes/src/darray.c lib/dtypes/src/graph.c src/astar.c  lib/dtypes/src/hset.c  lib/dtypes/src/prqueue.c -lm -march=native

vgraph_test:
	$(CC) $(CFLAGS) $(TEST) $(DTYPESSRC) $(ALGOSRC) src/test/vgraph_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_test
	$(MEMTEST) bin/vgraph_test

vgraph_perf_test:
	$(CC) $(CFLAGS) $(PERFFLAGS) $(DTYPESSRC) $(ALGOSRC) src/test/vgraph_test.c -I. -I$(DTYPESDIR) -o bin/vgraph_test
