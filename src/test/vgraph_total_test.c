#include "include/polygons.h"

#include "include/vgraph.h"
#include "include/astar.h"
#include "lib/dtypes/include/dtype.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

// =========================================================

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

MapPoint* random_point() {

  MapPoint *m = malloc(sizeof(MapPoint));
  m->x = (((double) rand()) / RAND_MAX) * 80 - 40;
  m->y = (((double) rand()) / RAND_MAX) * 80 - 40;

  return m;
}

void add_obstacles(DList *spheres, DList *polygons) {
 
  // First place a rectangle in the middle
  DList *corners = dlist_init();
  MapPoint *m;

  m = malloc(sizeof(MapPoint));
  m->x = -20;
  m->y = 10;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = 20;
  m->y = 10;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -20;
  m->y = -10;
  dlist_push(corners,m);

  m = malloc(sizeof(MapPoint));
  m->x = -20;
  m->y = -10;
  dlist_push(corners,m);

  PolygonalObstacle *p = malloc(sizeof(PolygonalObstacle));
  p->corners = corners;

  dlist_push(polygons,p);

  // Now some spheres

  CircularObstacle *c;

  c = obstacle_init(30,30,5);
  dlist_push(spheres,c);

  c = obstacle_init(-30,30,5);
  dlist_push(spheres,c);

  c = obstacle_init(30,-30,5);
  dlist_push(spheres,c);

  c = obstacle_init(-30,-30,5);
  dlist_push(spheres,c);

}

void random_polygons(DList *polygons) {

  const size_t N_POLYGONS = 30;
  const size_t MIN_CORNERS = 4;
  const size_t MAX_CORNERS = 9;

  const double WIDTH = 90;
  const double MIN_COORD = -45;
  const double SPREAD = 5.0;

  for(size_t ii = 0; ii < N_POLYGONS; ii++) {
    DList *base_points = dlist_init();
    MapPoint *m = malloc(sizeof(MapPoint));
    m->x = (((double) rand()) / RAND_MAX) * WIDTH + MIN_COORD;
    m->y = (((double) rand()) / RAND_MAX) * WIDTH + MIN_COORD;

    dlist_push(base_points, m);
    size_t n_corners = rand() % (MAX_CORNERS - MIN_CORNERS) + MIN_CORNERS;
    for(int jj = 0; jj < n_corners; jj++) {
      MapPoint *n = malloc(sizeof(MapPoint));
      n->x = ((double) rand()) / RAND_MAX * 2 * SPREAD - SPREAD + m->x;
      n->y = ((double) rand()) / RAND_MAX * 2 * SPREAD - SPREAD + m->y;
      dlist_push(base_points, n);
    }

    dlist_push(polygons, convex_hull(base_points));
  }

}

void print_graph_node(void* v) {

  MapPoint *m = (MapPoint*) v;
  printf("(%g,%g)", m->x, m->y);
}

// =========================================================

int main(int argc, const char** argv) {

  srand(time(0));

  DList *spheres = dlist_init(), *polygons = dlist_init();
  add_obstacles(spheres, polygons);

  MapPoint *start, *goal;
  start = malloc(sizeof(MapPoint));
  start->x = 50;
  start->y = 50;

  goal = malloc(sizeof(MapPoint));
  goal->x = -50;
  goal->y = -50;

  DList *points = dlist_init();
  for(size_t ii = 0; ii < 20; ii++) {
    dlist_push(points, random_point());
  }

  random_polygons(polygons);
  for(size_t idx = 0; idx < polygons->num_items; idx++) {
    printf("[\n");
    PolygonalObstacle *p = darray_get(polygons->data,idx);
    for(size_t corner_idx = 0; corner_idx < p->corners->num_items; corner_idx++) {
      MapPoint *corner = darray_get(p->corners->data, corner_idx);
      print_graph_node(corner);
      printf(",\n");
    }
    print_graph_node(darray_get(p->corners->data,0));
    printf("\n");
    printf("],\n");
  }

  PolygonalObstacle *hull = convex_hull(points);
  printf("Final path:\n");
  for(size_t idx = 0; idx < hull->corners->num_items; idx++) {
    print_graph_node(darray_get(hull->corners->data,idx));
    printf("\n");
  }
  print_graph_node(darray_get(hull->corners->data,0));
  printf("\n");

  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

  Graph *g = graph_init(GRAPH_DIRECTED);
  for(size_t idx = 0; idx < points->num_items; idx++) {
    graph_add(g,darray_get(points->data,idx));
  }
  graph_print(g,print_graph_node);
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

  return 0;
}
