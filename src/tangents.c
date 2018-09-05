#include "tangents.h"

#include <math.h>

DArray* tangent_circle_point(double point_x, double point_y, double circ_x, double circ_y, double circ_r) {


  // Point in the middle between P and center of circle
  double center_x = (point_x + circ_x) / 2;
  double center_y = (point_y + circ_y) / 2;


  // Radius of Thales circle
  double center_r = sqrt( (center_x - circ_x) * (center_x - circ_x) + (center_y - circ_y) * (center_y - circ_y) );


  // Intermediate value for calculating the polar for P
  //
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // This is the area where a fix needs to be applied
  double a = (circ_x * circ_x) + (circ_y * circ_y) - (circ_r * circ_r);
  a -= ( (center_x * center_x) + (center_y * center_y) - (center_r * center_r) );

  double div = circ_y - center_y;

  double m = (center_x - circ_x) / div;
  double n = a / (2.0 * div);

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  // x^2 + px + q = 0:
  
  a = (m*m + 1);

  double p = -2.0 * (circ_x - m * n + m * circ_y) / a;
  double q = (n * n + circ_x * circ_x - 2.0 * n * circ_y + circ_y * circ_y - circ_r * circ_r) / a;

  // Now solve the equation
  double sqrt_q = sqrt((p*p) / 4 - q);

  double x1 = -p/2 + sqrt_q, x2 = -p/2 - sqrt_q;
  double y1 = m * x1 + n, y2 = m * x2 + n;

  double t1_m = (point_y - y1) / (point_x - x1);
  double t1_n = y1 - t1_m * x1;

  double t2_m = (point_y - y2) / (point_x - x2);
  double t2_n = y2 - t2_m * x2;

  DArray *result = darray_init();

  LinearFunction *t1 = malloc(sizeof(LinearFunction));
  t1->m = t1_m;
  t1->n = t1_n;

  LinearFunction *t2 = malloc(sizeof(LinearFunction));
  t2->m = t2_m;
  t2->n = t2_n;

  darray_set(result, t1, 0);
  darray_set(result, t2, 1);

  return result;

}
