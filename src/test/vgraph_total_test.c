#include "include/polygons.h"

#include "include/vgraph.h"
#include "include/astar.h"
#include "lib/dtypes/include/dtype.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include <getopt.h>
#include <stdlib.h>
#include <unistd.h>

double euclid_distance(void *n1, void *n2) {

  MapPoint *m1 = (MapPoint*) n1;
  MapPoint *m2 = (MapPoint*) n2;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);
}


int main() {

  MapPoint *start = malloc(sizeof(MapPoint));
  start->x = 49.0;
  start->y = 50.0;

  MapPoint *goal = malloc(sizeof(MapPoint));
  goal->x = -100.0;
  goal->y = -100.0;

  CircularObstacle *c = obstacle_init(-50, -50, 5);

  MapPoint *p; DList *corners = dlist_init();

  p = malloc(sizeof(MapPoint));
  p->x = -20;
  p->y = -20;
  dlist_push(corners, p);

  p = malloc(sizeof(MapPoint));
  p->x = -30;
  p->y = -20;
  dlist_push(corners, p);

  p = malloc(sizeof(MapPoint));
  p->x = -30;
  p->y = -30;
  dlist_push(corners, p);

  p = malloc(sizeof(MapPoint));
  p->x = -20;
  p->y = -30;
  dlist_push(corners, p);

  PolygonalObstacle *o = malloc(sizeof(PolygonalObstacle));
  o->corners = corners;

  DList *spheres = dlist_init();
  dlist_push(spheres, c);

  DList *polygons = dlist_init();
  dlist_push(polygons, o);

  enum OBSTACLE_TYPES t;
  void *blocking = get_first_blocking(start, goal, spheres, polygons, NULL, euclid_distance, &t);

  printf("Polygon : %p\n", (void*) o);
  printf("Circle  : %p\n", (void*) c);
  printf("Blocking: %p\n", (void*) blocking);

  printf("Path blocked by %s!\n", (blocking == o) ? "Polygon" : "Circle");
}
