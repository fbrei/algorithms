#include "tangents.h"
#include "dtype.h"

#include <stdio.h>
#include <stdlib.h>

void print_results(DArray *out) {
  MapPoint *tmp = NULL;
  printf("Found the following points:\n");
  while(1) {
    tmp = (MapPoint*) darray_iterate(out, tmp);
    if(tmp == NULL) {
      break;
    }
    printf("Point: (%5.2lf, %5.2lf)\n", tmp->x, tmp->y);
  }
}

void example1() {

  printf("Tangent circle point\n");
  printf("====================\n");

  double point_x = 2;
  double point_y = 6;

  double circ_x = 9;
  double circ_y = 6;

  double circ_r = 3;

  DArray *out = tangent_circle_point_intersects(point_x, point_y, circ_x, circ_y, circ_r);

  if(out) {

    print_results(out);
    darray_destroy(out,free);
  }

  printf("\n");

}

void example2() {

  printf("Tangent circle circle\n");
  printf("=====================\n");

  DArray *out = tangent_circle_outer_intersects(2,3,3,8,9,3);

  if(out) {

    print_results(out);
    darray_destroy(out,free);
  }

}

int main(int argc, const char** argv) {

  example1();
  example2();

  return 0;
}
