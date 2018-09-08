#include "tangents.h"

#include <stdio.h>

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

DArray* tangent_circle_point_intersects(double point_x, double point_y, double circ_x, double circ_y, double circ_r) {


  // Point in the middle between P and center of circle
  double center_x = (point_x + circ_x) / 2;
  double center_y = (point_y + circ_y) / 2;


  // Radius of Thales circle
  double center_r = sqrt( (center_x - circ_x) * (center_x - circ_x) + (center_y - circ_y) * (center_y - circ_y) );


  double a = (circ_x * circ_x) + (circ_y * circ_y) - (circ_r * circ_r);
  a -= ( (center_x * center_x) + (center_y * center_y) - (center_r * center_r) );

  if(circ_y != center_y) {
    double div = circ_y - center_y;

    double m = (center_x - circ_x) / div;
    double n = a / (2.0 * div);

    // x^2 + px + q = 0:
    
    a = (m*m + 1);

    double p = -2.0 * (circ_x - m * n + m * circ_y) / a;
    double q = (n * n + circ_x * circ_x - 2.0 * n * circ_y + circ_y * circ_y - circ_r * circ_r) / a;
    double sqrt_q = sqrt((p*p) / 4 - q);

    double x1 = -p/2 + sqrt_q, x2 = -p/2 - sqrt_q;
    double y1 = m * x1 + n, y2 = m * x2 + n;

    DArray *result = darray_init();

    MapPoint *p1 = malloc(sizeof(MapPoint));
    p1->x = x1;
    p1->y = y1;

    MapPoint *p2 = malloc(sizeof(MapPoint));
    p2->x = x2;
    p2->y = y2;

    darray_set(result, p1, 0);
    darray_set(result, p2, 1);

    return result;
  } else {
  
    double x = -a / (2.0 * (center_x - circ_x));

    double p = -2.0 * circ_y;
    double q = (a*a) / (4.0 * (center_x - circ_x) * (center_x - circ_x)) + (a*circ_x) / (center_x - circ_x) + (circ_x*circ_x) \
               + (circ_y * circ_y) - (circ_r * circ_r);

    double sqrt_q = sqrt((p*p) / 4 - q);

    double y1 = -p/2 + sqrt_q, y2 = -p/2 - sqrt_q;

    DArray *result = darray_init();

    MapPoint *p1 = malloc(sizeof(MapPoint));
    p1->x = x;
    p1->y = y1;

    MapPoint *p2 = malloc(sizeof(MapPoint));
    p2->x = x;
    p2->y = y2;

    darray_set(result, p1, 0);
    darray_set(result, p2, 1);

    return result;
  
  }
}

DArray* tangent_circle_outer_intersects(double source_x, double source_y, double source_r,
    double target_x, double target_y, double target_r) {

  if(source_r == target_r) {
  
    DArray *result = darray_init();
    MapPoint *p1 = malloc(sizeof(MapPoint));
    MapPoint *p2 = malloc(sizeof(MapPoint));

    if(source_x != target_x) {

      double m = (target_y - source_y) / (target_x - source_x);
      double phi = atan(m);

      p1->x = -target_r * sin(phi) + target_x;
      p1->y = target_r * cos(phi) + target_y;

      p2->x = target_r * sin(phi) + target_x;
      p2->y = -target_r * cos(phi) + target_y;

    } else {

      p1->x = target_x - target_r;
      p1->y = target_y;

      p2->x = target_x + target_r;
      p2->y = target_y;
      
    }

    darray_set(result,p1,0);
    darray_set(result,p2,1);

    return result;
  
  }
  return NULL;
}

DArray* tangent_circle_inner_intersects(double source_x, double source_y, double source_r,
    double target_x, double target_y, double target_r) {

  if(source_r == target_r) {

    DArray *result = darray_init();
    MapPoint *p1 = malloc(sizeof(MapPoint));
    MapPoint *p2 = malloc(sizeof(MapPoint));


    double center_x = (source_x + target_x) / 2.0;
    double center_y = (source_y + target_y) / 2.0;

    double circ_x = (center_x + target_x) / 2.0;
    double circ_y = (center_y + target_y) / 2.0;

    double circ_r = sqrt((circ_x - target_x) * (circ_x - target_x) + (circ_y - target_y) * (circ_y - target_y));

    double a = (target_x * target_x - circ_x * circ_x);
    a += (target_y * target_y - circ_y * circ_y);
    a += (circ_r * circ_r  - target_r * target_r);

    if(source_x != target_x) {
      
      double m = (target_y - circ_y) / (circ_x - target_x);
      double n = -a / (2.0 * (circ_x - target_x));

      double div = (m*m + 1);

      double p = 2.0 * (m*n - m*target_x - target_y);
      double q = n*n - 2.0 * n * target_x + target_x * target_x + target_y * target_y - target_r * target_r;

      p /= div;
      q /= div;

      double sqrt_q = sqrt((p*p) / 4 - q);
      double y1 = -p / 2.0 + sqrt_q, x1 = m * y1 + n;
      double y2 = -p / 2.0 - sqrt_q, x2 = m * y2 + n;

      p1->x = x1;
      p1->y = y1;
      p2->x = x2;
      p2->y = y2;

    } else {
      
      double y = a / (2.0 * (target_y - circ_y));
      double p = -2.0 * target_x;
      double q = target_x * target_x + (y - target_y) * (y - target_y) - target_r * target_r;

      double sqrt_q = sqrt((p*p) / 4 - q);
      double x1 = -p / 2.0 + sqrt_q, y1 = y;
      double x2 = -p / 2.0 - sqrt_q, y2 = y;

      p1->x = x1;
      p1->y = y1;
      p2->x = x2;
      p2->y = y2;

    }

    darray_set(result,p1,0);
    darray_set(result,p2,1);

    return result;
  }

  return NULL;
}
