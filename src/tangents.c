#include "include/tangents.h"
#include "include/globals.h"

#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.141592653587932
#endif

#define NUM_INTERMEDIATES 3

double _get_arc_length(CircularObstacle *c, void *first_point, void *second_point);
void print_graph_node(void*);

DList* tangent_circle_point_intersects(MapPoint *p, CircularObstacle *c) {

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

    DList *result = dlist_init();

    MapPoint *p1 = malloc(sizeof(MapPoint));
    p1->x = x1;
    p1->y = y1;

    MapPoint *p2 = malloc(sizeof(MapPoint));
    p2->x = x2;
    p2->y = y2;

    dlist_push(result,p1);
    dlist_push(result,p2);

    return result;

  } else {
  
    double x = -a / (2.0 * (center_x - circ_x));

    double p = -2.0 * circ_y;
    double q = (a*a) / (4.0 * (center_x - circ_x) * (center_x - circ_x)) + (a*circ_x) / (center_x - circ_x) + (circ_x*circ_x) \
               + (circ_y * circ_y) - (circ_r * circ_r);

    double sqrt_q = sqrt((p*p) / 4 - q);

    double y1 = -p/2 + sqrt_q, y2 = -p/2 - sqrt_q;

    DList *result = dlist_init();

    MapPoint *p1 = malloc(sizeof(MapPoint));
    p1->x = x;
    p1->y = y1;

    MapPoint *p2 = malloc(sizeof(MapPoint));
    p2->x = x;
    p2->y = y2;

    dlist_push(result,p1);
    dlist_push(result,p2);

    return result;
  
  }
}

DList* tangent_circle_outer_intersects(CircularObstacle *c1, CircularObstacle *c2) {

  double source_x = c1->position.x;
  double source_y = c1->position.y;
  double source_r = c1->radius;

  double target_x = c2->position.x;
  double target_y = c2->position.y;
  double target_r = c2->radius;

  if(source_r == target_r) {
  
    DList *result = dlist_init();

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

      p1->x = source_x - target_r;
      p1->y = source_y;
      
      p2->x = target_x - target_r;
      p2->y = target_y;

      p3->x = source_x + target_r;
      p3->y = source_y;

      p4->x = target_x + target_r;
      p4->y = target_y;

    }

    dlist_push(result,p1);
    dlist_push(result,p2);
    dlist_push(result,p3);
    dlist_push(result,p4);


    return result;
  
  }
  return NULL;
}

DList* tangent_circle_inner_intersects(CircularObstacle *c1, CircularObstacle *c2) {

  double source_x = c1->position.x;
  double source_y = c1->position.y;
  double source_r = c1->radius;

  double target_x = c2->position.x;
  double target_y = c2->position.y;
  double target_r = c2->radius;

  if(source_r == target_r) {

    DList *result = dlist_init();

    double dx = source_x - target_x;
    double dy = source_y - target_y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < 2.0 * source_r) {
      return result;
    }

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
      double y_orig = (target_y + source_y) - y;
      double p = -2.0 * target_x;
      double q = target_x * target_x + (y - target_y) * (y - target_y) - target_r * target_r;

      double sqrt_q = sqrt((p*p) / 4 - q);
      double x1 = -p / 2.0 + sqrt_q, y1 = y;
      double x2 = -p / 2.0 - sqrt_q, y2 = y;

      p1->x = x2;
      p1->y = y_orig;

      p2->x = x1;
      p2->y = y1;

      p3->x = x1;
      p3->y = y_orig;

      p4->x = x2;
      p4->y = y2;

    }

    dlist_push(result,p1);
    dlist_push(result,p2);
    dlist_push(result,p3);
    dlist_push(result,p4);

    return result;
  }

  return NULL;
}

DList* tangent_circle_intersects(CircularObstacle *c1, CircularObstacle *c2) {
  DList *total = tangent_circle_outer_intersects(c1,c2);
  DList *second = tangent_circle_inner_intersects(c1,c2);


  MapPoint *tmp = NULL;
  while((tmp = dlist_iterate(second, tmp)) != NULL) {
    dlist_push(total,tmp);
  }
  dlist_destroy(second, NULL);

  return total;
}

unsigned short tangent_is_blocked(MapPoint *p1, MapPoint *p2, DList *obstacles) {

  void *tmp = NULL;

  while((tmp = dlist_iterate(obstacles, tmp)) != NULL) {
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

void _obstacle_connect_map_points(CircularObstacle *c, Graph *g, MapPoint *goal, double (*heuristic)(void*, void*), DList *other_obstacles) {

  (void) other_obstacles;
  _obstacle_sort_points(c, goal, heuristic);

  for(size_t ii = 0; ii < c->_num_map_points; ii++) {
    void *first = darray_get(c->_map_points, ii);
    void *second = darray_get(c->_map_points, (ii+1) % c->_num_map_points);

    double cost = _get_arc_length(c, first, second);
    if(goal != NULL && heuristic != NULL) {
      if(((MapPoint*) first)->h < ((MapPoint*) second)->h) {
        graph_connect(g, second, first, cost);
      } else {
        graph_connect(g, first, second, cost);
      }
    } else {
      graph_connect(g, first, second, cost);
      graph_connect(g, second, first, cost);
    }
  }
}

void _obstacle_connect_with_intermediate(CircularObstacle *c, Graph *g, MapPoint *goal, double (*heuristic)(void*, void*)) {

  _obstacle_sort_points(c, goal, heuristic);

  for(size_t ii = 0; ii < c->_num_map_points; ii++) {
    MapPoint *first = (MapPoint*) darray_get(c->_map_points, ii);
    MapPoint *second = (MapPoint*) darray_get(c->_map_points, (ii+1) % c->_num_map_points);

    double r = c->radius;
    double x0 = c->position.x;
    double y0 = c->position.y;

    double first_angle = first->score / r;
    double second_angle = second->score / r;

    double anglediff = fabs(second_angle - first_angle);


    if(anglediff > M_PI) {
      anglediff = 2.0 * M_PI - anglediff;
    }
    double d_angle = anglediff / (NUM_INTERMEDIATES + 1);

    MapPoint ** intermediates = malloc(NUM_INTERMEDIATES * sizeof(MapPoint*));
    intermediates[0] = malloc(sizeof(MapPoint));
    intermediates[0]->x = r * cos(first_angle + d_angle) + x0;
    intermediates[0]->y = r * sin(first_angle + d_angle) + y0;
    double cost = heuristic(first, intermediates[0]);

    graph_add(g, intermediates[0]);
    graph_connect(g, first, intermediates[0], cost);
    graph_connect(g, intermediates[0], first, cost);

#if NUM_INTERMEDIATES > 1
    intermediates[NUM_INTERMEDIATES - 1] = malloc(sizeof(MapPoint));
    intermediates[NUM_INTERMEDIATES - 1]->x = r * cos(second_angle - d_angle) + x0;
    intermediates[NUM_INTERMEDIATES - 1]->y = r * sin(second_angle - d_angle) + y0;
    graph_add(g, intermediates[NUM_INTERMEDIATES-1]);
#endif

    graph_connect(g, intermediates[NUM_INTERMEDIATES-1], second, heuristic(intermediates[NUM_INTERMEDIATES-1], second));
    graph_connect(g, second, intermediates[NUM_INTERMEDIATES-1], heuristic(intermediates[NUM_INTERMEDIATES-1], second));


    for(size_t ii = 1; ii < (NUM_INTERMEDIATES - 1); ii++) {
      intermediates[ii] = malloc(sizeof(MapPoint));
      intermediates[ii]->x = r * cos(first_angle + ii * d_angle) + x0;
      intermediates[ii]->y = r * sin(first_angle + ii * d_angle) + y0;
      graph_add(g,intermediates[ii]);
      if(heuristic(goal, intermediates[ii]) < heuristic(goal, intermediates[ii-1])) {
        graph_connect(g, intermediates[ii-1], intermediates[ii], cost);
      } else {
        graph_connect(g, intermediates[ii], intermediates[ii-1], cost);
      }
    }

#if NUM_INTERMEDIATES > 1
      graph_connect(g, intermediates[NUM_INTERMEDIATES - 2], intermediates[NUM_INTERMEDIATES - 1], cost);
      graph_connect(g, intermediates[NUM_INTERMEDIATES - 1], intermediates[NUM_INTERMEDIATES - 2], cost);
#endif

  }

}
CircularObstacle* tangent_get_first_blocking(MapPoint *from, MapPoint *to, DList *obstacles, void *self, double (*distance)(void*,void*)) {

  CircularObstacle *closest_obstacle = NULL;
  double closest_dist = 999999;

  void *tmp = NULL;
  while((tmp = dlist_iterate(obstacles, tmp)) != NULL) {
    CircularObstacle *co = (CircularObstacle*) tmp;
    if(tmp == self) {
      continue;
    }

    MapPoint *po = &(co->position);

    double x = co->position.x;
    double y = co->position.y;

    double r = 0.99 * co->radius;

    double m = (from->y - to->y) / (from->x - to->x);
    double n = -from->x * m + from->y;

    double p = (2.0 * (m*n - x - m * y)) / (m * m + 1);
    double q = (n*n + x * x + y * y - 2 * n * y - r * r) / (m * m + 1);

    if( (p*p)/4 > q ) {

      double min_x = (from->x < to->x) ? from->x : to->x;
      double min_y = (from->y < to->y) ? from->y : to->y;

      double max_x = (from->x < to->x) ? to->x : from->x;
      double max_y = (from->y < to->y) ? to->y : from->y;

      double sqrt_q = sqrt( (p*p)/4.0 - q  );

      double x1 = -p / 2.0 + sqrt_q, y1 = m * x1 + n;
      double x2 = -p / 2.0 + sqrt_q, y2 = m * x2 + n;

      if( (x1 > min_x && x1 < max_x && y1 > min_y && y1 < max_y) || (x2 > min_x && x2 < max_x && y2 > min_y && y2 < max_y) ) {
        double new_dist = distance(po, from);
        if(new_dist < closest_dist) {
          closest_dist = new_dist;
          closest_obstacle = co;
        }
      } 

    }
  }

  return closest_obstacle;
}

DList* tangent_get_blocking(MapPoint *from, MapPoint *to, DList *obstacles, void *self) {

  void *tmp = NULL;
  DList* results = NULL;
  /* while((tmp = dlist_iterate(obstacles, tmp)) != NULL) { */
  for(size_t idx = 0; idx < obstacles->num_items; idx++) {
    tmp = darray_get(obstacles->data,idx);
    
    CircularObstacle *co = (CircularObstacle*) tmp;
    if(tmp == self) {
      continue;
    }

    double x = co->position.x;
    double y = co->position.y;

    double r = 0.99 * co->radius;

    double m = (from->y - to->y) / (from->x - to->x);
    double n = -from->x * m + from->y;

    double p = (2.0 * (m*n - x - m * y)) / (m * m + 1);
    double q = (n*n + x * x + y * y - 2 * n * y - r * r) / (m * m + 1);

    if( (p*p)/4 > q ) {

      double min_x = (from->x < to->x) ? from->x : to->x;
      /* double min_y = (from->y < to->y) ? from->y : to->y; */

      double max_x = (from->x < to->x) ? to->x : from->x;
      /* double max_y = (from->y < to->y) ? to->y : from->y; */

      double sqrt_q = sqrt( (p*p)/4.0 - q  );

      double x1 = -p / 2.0 + sqrt_q /* , y1 = m * x1 + n */ ;
      double x2 = -p / 2.0 + sqrt_q /* , y2 = m * x2 + n */ ;

      /* if( (x1 > min_x && x1 < max_x && y1 > min_y && y1 < max_y) || (x2 > min_x && x2 < max_x && y2 > min_y && y2 < max_y) ) { */
      if( (x1 > min_x && x1 < max_x) || (x2 > min_x && x2 < max_x)) {
        if(results == NULL) {
          results = dlist_init();
        }
        dlist_push(results, tmp);
      } 

    }
  }

  return results;
}

void _obstacle_connect_directed_intermediates(CircularObstacle *c, Graph *g, MapPoint *goal, double (*heuristic)(void*, void*)) {

  size_t num_nodes = c->_map_points->num_items;

  if(num_nodes == 0) {
    return;
  }

  _obstacle_sort_points(c, goal, heuristic);

  for(size_t ii = 0; ii < c->_num_map_points; ii++) {
    MapPoint *current = (MapPoint*) darray_get(c->_map_points,ii);
    if(current->is_in == 0) continue;
    double score_cw = 0.0, score_ccw = 0.0;
    MapPoint *best_cw = NULL, *best_ccw = NULL;

    // Find closest exit in counter-clockwise direction
    for(size_t jj = (ii+1) % num_nodes; jj < c->_num_map_points; jj = (jj+1) % num_nodes)  {
      if(ii == jj) continue;
      MapPoint *other = (MapPoint*) darray_get(c->_map_points,jj);
      if(other->is_in == 1) continue;
      score_ccw = fabs(_get_arc_length(c, current, other));
      best_ccw = other;
      break;
    }

    // Find closest exit in clockwise direction
    for(size_t jj = (ii == 0) ? num_nodes - 1 : ii - 1; jj < c->_num_map_points; jj = (jj == 0) ? num_nodes - 1 : jj-1 )  {
      if(ii == jj) continue;
      MapPoint *other = (MapPoint*) darray_get(c->_map_points,jj);
      if(other->is_in == 1) continue;
      score_cw = fabs(_get_arc_length(c, current, other));
      best_cw = other;
      break;
    }

    if(score_cw < score_ccw) {
      graph_connect(g, current, best_cw, score_cw);
    } else {
      graph_connect(g, current, best_ccw, score_ccw);
    }
  }
}

void _obstacle_sort_points(CircularObstacle *c, MapPoint *goal, double (*heuristic)(void*, void*)) {

  void *tmp = NULL;
  MapPoint origin = { .x = c->position.x + c->radius, .y = c->position.y };
  while((tmp = darray_iterate(c->_map_points, tmp)) != NULL) {

    MapPoint *m = (MapPoint*) tmp;
    m->score = _get_arc_length(c,&origin,m);
    if(heuristic != NULL && goal != NULL) {
      m->h = heuristic((void*) goal, m);
    }
    if(m->y < c->position.y) {
      m->score = 2.0 * M_PI * c->radius - m->score;
    }
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

}

double _get_arc_length(CircularObstacle *c, void *first_point, void *second_point) {

  MapPoint *m1 = (MapPoint*) first_point;
  MapPoint *m2 = (MapPoint*) second_point;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;
  double straight = sqrt(dx*dx + dy*dy);

  double angle;
  if(fabs(straight - 2.0 * c->radius) < 0.0001) {
    angle = M_PI;
  } else {
    angle = 2 * asin(straight / (2.0 * c->radius));
  }

  return angle * c->radius;
}
