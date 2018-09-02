astar_test:
	gcc -std=c11 -o bin/astar_test -Iinclude -Ilib/dtypes/include src/astar.c src/test/astar_test.c
	valgrind bin/astar_test
