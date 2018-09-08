#include "tangents.h"
#include "dtype.h"

#include <stdio.h>
#include <stdlib.h>

void print_results(DArray *out) {
  MapPoint *tmp = NULL;
  while(1) {
    tmp = (MapPoint*) darray_iterate(out, tmp);
    if(tmp == NULL) {
      break;
    }
    printf("(%5.2lf, %5.2lf) -> ", tmp->x, tmp->y);
    tmp = (MapPoint*) darray_iterate(out, tmp);
    printf("(%5.2lf, %5.2lf)\n", tmp->x, tmp->y);
  }
}

void example1() {

  printf("Tangent circle point\n");
  printf("====================\n");

  double point_x = -6;
  double point_y = -8;

  double circ_x = -3;
  double circ_y = -5;

  double circ_r = 2;

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

  double m_x = -3.0, m_y = -5.0, m_r = 2.0;
  double n_x = 10.0, n_y = 3.0, n_r = 2.0;

  printf("(x-%g)^2 + (y-%g)^2 = %g^2\n",m_x,m_y,m_r);
  printf("(x-%g)^2 + (y-%g)^2 = %g^2\n",n_x,n_y,n_r);


  printf("Outer tangent points:\n");
  DArray *out = tangent_circle_outer_intersects(m_x,m_y,m_r,n_x,n_y,n_r);

  if(out) {

    print_results(out);
    darray_destroy(out,free);
  }

  printf("Inner tangent points:\n");
  out = tangent_circle_inner_intersects(m_x,m_y,m_r,n_x,n_y,n_r);

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
