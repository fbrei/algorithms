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

void print_graph_node(void* v) {

  MapPoint *m = (MapPoint*) v;
  printf("(%g,%g)", m->x, m->y);
}

double euclid_distance(void *n1, void *n2) {

  MapPoint *m1 = (MapPoint*) n1;
  MapPoint *m2 = (MapPoint*) n2;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);
}

void dump_polygons(DList *polygons) {

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
}

union hash_u {
  double x;
  unsigned long n;
};

unsigned long hash(void *a) {
  MapPoint *m = (MapPoint*) a;

  union hash_u h1;
  h1.x = m->x;

  union hash_u h2;
  h2.x = m->y;

  return (h1.n & 0xFFFF0000) | (h2.n & 0x0000FFFF);
}

unsigned int sphere_equals(void *a, void *b) {

  MapPoint *m = (MapPoint*) a;
  MapPoint *n = (MapPoint*) b;

  return m->x == n->x && m->y == n->y;
}

int main() {

  MapPoint *start = malloc(sizeof(MapPoint));
  start->x = 49.0;
  start->y = 50.0;
  start->obstacle = NULL;

  MapPoint *goal = malloc(sizeof(MapPoint));
  goal->x = -50.0;
  goal->y = -50.0;
  goal->obstacle = NULL;


  PolygonalObstacle *o = malloc(sizeof(PolygonalObstacle));
  MapPoint *p; DList *corners = dlist_init();

  p = malloc(sizeof(MapPoint));
  p->x = -20;
  p->y = -20;
  p->obstacle = o;
  p->on_circle = 0;
  dlist_push(corners, p);

  p = malloc(sizeof(MapPoint));
  p->x = -30;
  p->y = -20;
  p->obstacle = o;
  p->on_circle = 0;
  dlist_push(corners, p);

  p = malloc(sizeof(MapPoint));
  p->x = -30;
  p->y = -30;
  p->obstacle = o;
  p->on_circle = 0;
  dlist_push(corners, p);

  p = malloc(sizeof(MapPoint));
  p->x = -20;
  p->y = -30;
  p->obstacle = o;
  p->on_circle = 0;
  dlist_push(corners, p);

  o->corners = corners;

  DList *spheres = dlist_init();

  CircularObstacle *c = obstacle_init(25, 25, 5);
  c->points = hset_init(hash, sphere_equals);
  fprintf(stderr, "   C1: %p\n", (void*) c);
  dlist_push(spheres, c);

  c = obstacle_init(0, 10, 5);
  c->points = hset_init(hash, sphere_equals);
  fprintf(stderr, "   C2: %p\n", (void*) c);
  dlist_push(spheres, c);

  c = obstacle_init(-10, -24, 5);
  c->points = hset_init(hash, sphere_equals);
  fprintf(stderr, "   C3: %p\n", (void*) c);
  dlist_push(spheres, c);

  DList *polygons = dlist_init();
  dlist_push(polygons, o);

  fprintf(stderr, "   P1: %p\n", (void*) o);

  clock_t t_mn8oe4cu, t_v9mbndsz;
  
  t_mn8oe4cu = clock();
  Graph *g = vgraph(start, goal, polygons, spheres, euclid_distance, VGRAPH_WITH_SPHERES, 2, NULL);
  AStarPathNode *ap = astar(g, start, goal, euclid_distance, NULL);
  t_v9mbndsz = clock();
  
  double total_time = ((double) (t_v9mbndsz - t_mn8oe4cu)) / CLOCKS_PER_SEC;
  printf("Time taken: %gs [%luus]\n", total_time, t_v9mbndsz - t_mn8oe4cu);

  printf("Path length: %g\n", (ap != NULL) ? ap->total_dist : 999999.9);

  printf("Final Path:\n");
  while(ap) {
    print_graph_node(ap->data);
    printf("\n");
    ap = ap->parent;
  }
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

  graph_print(g, print_graph_node);

  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
}
