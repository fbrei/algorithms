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

  clock_t t_mn8oe4cu, t_v9mbndsz;
  
  t_mn8oe4cu = clock();
  Graph *g = vgraph(start, goal, polygons, spheres, euclid_distance, VGRAPH_WITH_SPHERES, 0, NULL);
  AStarPathNode *ap = astar(g, start, goal, euclid_distance, NULL);
  t_v9mbndsz = clock();
  
  double total_time = ((double) (t_v9mbndsz - t_mn8oe4cu)) / CLOCKS_PER_SEC;
  printf("Time taken: %gs\n", total_time);

  printf("Path length: %g\n", ap->total_dist);

  graph_print(g, print_graph_node);

}
