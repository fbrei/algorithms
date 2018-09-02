astar_test:
	clang -std=c11 -O0 -ggdb -o bin/astar_test -Iinclude -Ilib/dtypes/include src/astar.c src/test/astar_test.c lib/dtypes/src/darray.c lib/dtypes/src/prqueue.c lib/dtypes/src/graph.c -lm 
	valgrind --tool=memcheck --leak-check=full bin/astar_test
