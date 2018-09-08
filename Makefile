all: astar_test tangent_test

astar_test:
	clang -std=c11 -O0 -ggdb -o bin/astar_test -Iinclude -Ilib/dtypes/include src/astar.c src/test/astar_test.c lib/dtypes/src/darray.c lib/dtypes/src/prqueue.c lib/dtypes/src/graph.c lib/dtypes/src/hset.c -lm 
	valgrind --tool=memcheck --leak-check=full bin/astar_test

tangent_test:
	clang -std=c11 -O0 -ggdb -o bin/tangent_test -Iinclude -Ilib/dtypes/include src/test/tangent_test.c src/tangents.c lib/dtypes/src/darray.c -lm
	valgrind --tool=memcheck --leak-check=full bin/tangent_test

perf_test:
	clang -std=c11 -O3 -o bin/tangent_test -Iinclude -Ilib/dtypes/include src/test/tangent_test.c src/tangents.c lib/dtypes/src/darray.c -lm -march=native
