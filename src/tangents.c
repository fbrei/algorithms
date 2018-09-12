#include "tangents.h"

#include <stdio.h>
#include <math.h>

#define M_PI 3.141592653587932

double _get_arc_length(CircularObstacle *c, void *first_point, void *second_point);

DArray* tangent_circle_point_intersects(MapPoint *p, CircularObstacle *c) {

  double point_x = p->x;
  double point_y = p->y;

  double circ_x = c->position.x;
  double circ_y = c->position.y;
  double circ_r = c->radius;

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

DArray* tangent_circle_outer_intersects(CircularObstacle *c1, CircularObstacle *c2) {

  double source_x = c1->position.x;
  double source_y = c1->position.y;
  double source_r = c1->radius;

  double target_x = c2->position.x;
  double target_y = c2->position.y;
  double target_r = c2->radius;

  if(source_r == target_r) {
  
    DArray *result = darray_init();
    MapPoint *p1 = malloc(sizeof(MapPoint));
    MapPoint *p2 = malloc(sizeof(MapPoint));
    MapPoint *p3 = malloc(sizeof(MapPoint));
    MapPoint *p4 = malloc(sizeof(MapPoint));

    if(source_x != target_x) {

      double m = (target_y - source_y) / (target_x - source_x);
      double phi = atan(m);

      p1->x = -target_r * sin(phi) + source_x;
      p1->y = target_r * cos(phi) + source_y;

      p3->x = target_r * sin(phi) + source_x;
      p3->y = -target_r * cos(phi) + source_y;

      p2->x = -target_r * sin(phi) + target_x;
      p2->y = target_r * cos(phi) + target_y;

      p4->x = target_r * sin(phi) + target_x;
      p4->y = -target_r * cos(phi) + target_y;

    } else {

      p1->x = target_x - target_r;
      p1->y = target_y;

      p2->x = target_x + target_r;
      p2->y = target_y;

      p3->x = 0;
      p3->y = 0;
      p4->x = 0;
      p4->y = 0;
      
    }

    darray_set(result,p1,0);
    darray_set(result,p2,1);
    darray_set(result,p3,2);
    darray_set(result,p4,3);

    return result;
  
  }
  return NULL;
}

DArray* tangent_circle_inner_intersects(CircularObstacle *c1, CircularObstacle *c2) {

  double source_x = c1->position.x;
  double source_y = c1->position.y;
  double source_r = c1->radius;

  double target_x = c2->position.x;
  double target_y = c2->position.y;
  double target_r = c2->radius;

  if(source_r == target_r) {

    DArray *result = darray_init();
    MapPoint *p1 = malloc(sizeof(MapPoint));
    MapPoint *p2 = malloc(sizeof(MapPoint));
    MapPoint *p3 = malloc(sizeof(MapPoint));
    MapPoint *p4 = malloc(sizeof(MapPoint));


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

      p1->x = 2.0 * center_x - x1;
      p1->y = 2.0 * center_y - y1;

      p2->x = x1;
      p2->y = y1;

      p3->x = 2.0 * center_x - x2;
      p3->y = 2.0 * center_y - y2;

      p4->x = x2;
      p4->y = y2;

    } else {
      
      double y = a / (2.0 * (target_y - circ_y));
      double p = -2.0 * target_x;
      double q = target_x * target_x + (y - target_y) * (y - target_y) - target_r * target_r;

      double sqrt_q = sqrt((p*p) / 4 - q);
      double x1 = -p / 2.0 + sqrt_q, y1 = y;
      double x2 = -p / 2.0 - sqrt_q, y2 = y;

      p2->x = x1;
      p2->y = y1;
      p4->x = x2;
      p4->y = y2;

    }

    darray_set(result,p1,0);
    darray_set(result,p2,1);
    darray_set(result,p3,2);
    darray_set(result,p4,3);

    return result;
  }

  return NULL;
}

unsigned short tangent_is_blocked(MapPoint *p1, MapPoint *p2, DArray *obstacles) {

  void *tmp = NULL;

  while((tmp = darray_iterate(obstacles, tmp)) != NULL) {
    CircularObstacle *co = (CircularObstacle*) tmp;

    double x = co->position.x;
    double y = co->position.y;

    double r = 0.99 * co->radius;

    double m = (p1->y - p2->y) / (p1->x - p2->x);
    double n = -p1->x * m + p1->y;

    double p = (2.0 * (m*n - x - m * y)) / (m * m + 1);
    double q = (n*n + x * x + y * y - 2 * n * y - r * r) / (m * m + 1);

    if( (p*p)/4 > q ) {

      double min_x = (p1->x < p2->x) ? p1->x : p2->x;
      double min_y = (p1->y < p2->y) ? p1->y : p2->y;

      double max_x = (p1->x < p2->x) ? p2->x : p1->x;
      double max_y = (p1->y < p2->y) ? p2->y : p1->y;

      double sqrt_q = sqrt( (p*p)/4.0 - q  );

      double x1 = -p / 2.0 + sqrt_q, y1 = m * x1 + n;
      double x2 = -p / 2.0 + sqrt_q, y2 = m * x2 + n;

      if( (x1 > min_x && x1 < max_x && y1 > min_y && y1 < max_y)
       || (x2 > min_x && x2 < max_x && y2 > min_y && y2 < max_y) ) {
        return 1;
      } 

    }
  }

  return 0;
}

CircularObstacle* obstacle_init(double x, double y, double r) {

  CircularObstacle *c = malloc(sizeof(CircularObstacle));
  c->position.x = x;
  c->position.y = y;
  c->radius = r;

  c->_num_map_points = 0;
  c->_map_points = darray_init();

  return c;

}

void obstacle_destroy(void *c) {

  CircularObstacle *co = (CircularObstacle*) c;
  darray_destroy(co->_map_points, NULL);
  free(co);
}

void _obstacle_add_map_point(CircularObstacle *c, MapPoint *p) {

  darray_set(c->_map_points, p, c->_num_map_points);
  c->_num_map_points++;

}

void _obstacle_connect_map_points(CircularObstacle *c, Graph *g) {
  
  void *tmp = NULL;
  while((tmp = darray_iterate(c->_map_points, tmp)) != NULL) {

    MapPoint *m = (MapPoint*) tmp;
    m->score = acos((m->x - c->position.x) / c->radius);
    if(m->y < c->position.y) m->score = -(m->score);
  }

  for(size_t ii = 0; ii < c->_num_map_points; ii++) {
    for(size_t jj = 0; jj < c->_num_map_points - (ii + 1); jj++) {
      MapPoint *m1 = (MapPoint*) darray_get(c->_map_points, jj);
      MapPoint *m2 = (MapPoint*) darray_get(c->_map_points, jj+1);
      if(m1->score > m2->score) {
        darray_set(c->_map_points, m2, jj);
        darray_set(c->_map_points, m1, jj+1);
      }
    }
  }

  for(size_t ii = 0; ii < c->_num_map_points; ii++) {
    void *first = darray_get(c->_map_points, ii);
    void *second = darray_get(c->_map_points, (ii+1) % c->_num_map_points);

    double cost = _get_arc_length(c, first, second);
    graph_connect(g, first, second, cost);
    graph_connect(g, second, first, cost);
  }

}

double _get_arc_length(CircularObstacle *c, void *first_point, void *second_point) {

  MapPoint *m1 = (MapPoint*) first_point;
  MapPoint *m2 = (MapPoint*) second_point;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);

}
