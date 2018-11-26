#include "include/polygons.h"

#include "include/vgraph.h"
#include "include/astar.h"
#include "lib/dtypes/include/dtype.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

/**
 * Calculates the euclidian distance between two map points
 * that must be given as void pointers for compatibility
 */
double euclid_distance(void *n1, void *n2) {

  MapPoint *m1 = (MapPoint*) n1;
  MapPoint *m2 = (MapPoint*) n2;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);
}

unsigned long hash(void *n) {
  MapPoint *m = (MapPoint*) n;

  union longdouble {
    unsigned long out;
    double in;
  } a,b ;

  a.in = m->x;
  b.in = m->y;

  return (a.out & 0xFFFF) | (b.out & 0xFFFF0000);
}

void print_graph_node(void* v) {

  MapPoint *m = (MapPoint*) v;
  printf("(%g,%g)", m->x, m->y);
}

void add_obstacles(DList *obstacles) {
  DList *corners;
  MapPoint *m;
  PolygonalObstacle *o;

  // =====================================0

  corners = dlist_init();

  m = malloc(sizeof(MapPoint));
  m->x = 15;
  m->y = 10;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = 5;
  m->y = 10;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = 5;
  m->y = -10;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = 15;
  m->y = -10;
  dlist_push(corners,m);
  
  o = malloc(sizeof(PolygonalObstacle));
  o->corners = corners;

  dlist_push(obstacles,o);

  // =====================================0

  corners = dlist_init();

  m = malloc(sizeof(MapPoint));
  m->x = -20;
  m->y = 30;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -40;
  m->y = 30;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -40;
  m->y = 20;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -20;
  m->y = 20;
  dlist_push(corners,m);
  
  o = malloc(sizeof(PolygonalObstacle));
  o->corners = corners;

  dlist_push(obstacles,o);

  // =====================================0

  corners = dlist_init();

  m = malloc(sizeof(MapPoint));
  m->x = -33;
  m->y = -15;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -43;
  m->y = -25;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -23;
  m->y = -25;
  dlist_push(corners,m);

  o = malloc(sizeof(PolygonalObstacle));
  o->corners = corners;

  dlist_push(obstacles,o);
}

// =======================================================

int main(int argc, const char** argv) {

  DList *obstacles = dlist_init();
  add_obstacles(obstacles);

  MapPoint *start, *goal;
  start = malloc(sizeof(MapPoint));
  start->x = 50;
  start->y = 50;

  goal = malloc(sizeof(MapPoint));
  goal->x = -50;
  goal->y = -50;

  clock_t t1, t_inter, t2;
  AStarPathNode *p;

  t1 = clock();
  Graph *g = vgraph_polygonal_obstacles(start,goal,obstacles,euclid_distance,0);
  t_inter = clock();
  p = astar(g, start, goal, euclid_distance, hash);
  t2 = clock();

  fprintf(stderr, "Time spent: %luus\n", t2 - t1);

  while(p) {
    print_graph_node(p->data);
    printf("\n");
    p = p->parent;
  }

  graph_print(g,print_graph_node);

  return 0;
}
