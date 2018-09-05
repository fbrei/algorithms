#include "tangents.h"
#include "dtype.h"

#include <stdio.h>
#include <stdlib.h>


int main(int argc, const char** argv) {

  double point_x = 2;
  double point_y = 6;

  double circ_x = 9;
  double circ_y = 6;

  double circ_r = 3;

  DArray *out = tangent_circle_point_intersects(point_x, point_y, circ_x, circ_y, circ_r);

  MapPoint *tmp = NULL;

  printf("Found the following tangents:\n");
  while(1) {
    tmp = (MapPoint*) darray_iterate(out, tmp);
    if(tmp == NULL) {
      break;
    }
    printf("Point: (%5.2lf, %5.2lf)\n", tmp->x, tmp->y);
  }

  darray_destroy(out, free);

  return 0;
}
