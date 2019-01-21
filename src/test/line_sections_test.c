#include "include/globals.h"
#include "include/polygons.h"

#include <stdio.h>

void test_two_lines(MapPoint *from_a, MapPoint *to_a, MapPoint *from_b, MapPoint *to_b, unsigned short expected, char* descr) {

  unsigned short res = _lines_intersect(from_a, to_a, from_b, to_b);
  if(res == expected) {
    printf("[success] ");
  } else {
    printf("[FAIL]    ");
  }
  printf("Testing (%g,%g) -> (%g,%g) | (%g,%g) -> (%g,%g) => Expected: %u | Received %u (%s)\n", from_a->x, from_a->y, to_a->x, to_a->y, from_b->x, from_b->y, to_b->x, to_b->y, expected, res, descr) ;
}

int main(int argc, const char** argv) {

  UNUSED(argc);
  UNUSED(argv);

  MapPoint *from_a, *to_a, *from_b, *to_b;
  
  from_a = malloc(sizeof(MapPoint));
  to_a = malloc(sizeof(MapPoint));
  from_b = malloc(sizeof(MapPoint));
  to_b = malloc(sizeof(MapPoint));

  from_a->x = 10, from_a->y = 10, to_a->x = 20, to_a->y = 20;
  from_b->x = 10, from_b->y = 15, to_b->x = 25, to_b->y = 15;

  test_two_lines(from_a, to_a, from_b, to_b, 1, "Two diagonals");

  from_a->x = 10, from_a->y = 10, to_a->x = 20, to_a->y = 20;
  from_b->x = 20, from_b->y = 20, to_b->x = 25, to_b->y = 25;

  test_two_lines(from_a, to_a, from_b, to_b, 1, "Common end point");

  from_a->x = 10, from_a->y = 10, to_a->x = 10, to_a->y = 20;
  from_b->x = 5, from_b->y = 15, to_b->x = 25, to_b->y = 15;

  test_two_lines(from_a, to_a, from_b, to_b, 1, "One vertical");

  from_a->x = 10, from_a->y = 15, to_a->x = 25, to_a->y = 15;
  from_b->x = 10, from_b->y = 10, to_b->x = 25, to_b->y = 25;

  test_two_lines(from_a, to_a, from_b, to_b, 1, "One horizontal");

  from_a->x = 10, from_a->y = 10, to_a->x = 10, to_a->y = 20;
  from_b->x = 13, from_b->y = 10, to_b->x = 13, to_b->y = 20;

  test_two_lines(from_a, to_a, from_b, to_b, 0, "Parallel verticals");

  return 0;
}
