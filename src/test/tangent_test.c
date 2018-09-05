#include "tangents.h"
#include "dtype.h"

#include <stdio.h>
#include <stdlib.h>


int main(int argc, const char** argv) {

  double point_x = 9;
  double point_y = 20;

  double circ_x = 9;
  double circ_y = 6;

  double circ_r = 3;

  DArray *out = tangent_circle_point(point_x, point_y, circ_x, circ_y, circ_r);

  LinearFunction *tmp = NULL;

  printf("Found the following tangents:\n");
  while(1) {
    tmp = (LinearFunction*) darray_iterate(out, tmp);
    if(tmp == NULL) {
      break;
    }
    printf("y = %5.2lfx + %5.2lf\n", tmp->m, tmp->n);
  }

  darray_destroy(out, free);

  return 0;
}
