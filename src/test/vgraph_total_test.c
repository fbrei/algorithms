#include "include/polygons.h"

#include "include/vgraph.h"
#include "include/astar.h"
#include "lib/dtypes/include/dtype.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

// =========================================================

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

void random_polygons(DList *polygons, const size_t N_POLYGONS) {

  const size_t MIN_CORNERS = 4;
  const size_t MAX_CORNERS = 9;

  const double WIDTH = 90;
  const double MIN_COORD = -45;
  const double SPREAD = 1.0;

  for(size_t ii = 0; ii < N_POLYGONS; ii++) {
    DList *base_points = dlist_init();
    MapPoint *m = malloc(sizeof(MapPoint));
    m->x = (((double) rand()) / RAND_MAX) * WIDTH + MIN_COORD;
    m->y = (((double) rand()) / RAND_MAX) * WIDTH + MIN_COORD;

    dlist_push(base_points, m);
    size_t n_corners = rand() % (MAX_CORNERS - MIN_CORNERS) + MIN_CORNERS;
    for(size_t jj = 0; jj < n_corners; jj++) {
      MapPoint *n = malloc(sizeof(MapPoint));
      n->x = ((double) rand()) / RAND_MAX * 2 * SPREAD - SPREAD + m->x;
      n->y = ((double) rand()) / RAND_MAX * 2 * SPREAD - SPREAD + m->y;
      dlist_push(base_points, n);
    }

    dlist_push(polygons, convex_hull(base_points));
  }

}


void evil_test_set(DList *polygons) {

  DList *points = dlist_init();
  MapPoint *m;

  m = malloc(sizeof(MapPoint));
  m->x = 10;
  m->y = 30;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = 30;
  m->y = 10;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = 10;
  m->y = -10;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = -10;
  m->y = 10;
  dlist_push(points,m);

  PolygonalObstacle *p = malloc(sizeof(PolygonalObstacle));
  p->corners = points;
  dlist_push(polygons,p);

}

// Merge overlapping polygons into an and create
// a new convex hull
DList* merge_polygons(DList* polygons) {
  unsigned short merged = 1;

  DList *new_polygons = polygons;

  while(merged) {
    merged = 0;

    for(size_t ii = 0; ii < new_polygons->num_items; ii++) {
      PolygonalObstacle *p = darray_get(new_polygons->data, ii);

      for(size_t jj = 0; jj < p->corners->num_items; jj++) {
        MapPoint *from = darray_get(p->corners->data,jj);
        MapPoint *to = darray_get(p->corners->data,(jj+1) % p->corners->num_items);

        PolygonalObstacle *o = polygon_get_first_blocking(from,to,new_polygons,p);
        if(o != NULL) {
          DList *points = dlist_init();
          for(size_t kk = 0; kk < p->corners->num_items; kk++) {
            dlist_push(points, darray_get(p->corners->data,kk));
          }
          for(size_t kk = 0; kk < o->corners->num_items; kk++) {
            dlist_push(points, darray_get(o->corners->data,kk));
          }

          PolygonalObstacle *tmp = convex_hull(points);
          DList *new_new_polygons = dlist_init();
          dlist_push(new_new_polygons, tmp);

          for(size_t kk = 0; kk < new_polygons->num_items; kk++) {
            PolygonalObstacle *l = darray_get(new_polygons->data, kk);
            if(l != o && l != p)
              dlist_push(new_new_polygons,l);
          }
          new_polygons = new_new_polygons;
          merged = 1;

          break;
        }
      }
      if(merged) break;
    }
  }

  return new_polygons;
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

// =========================================================

int main(int argc, const char** argv) {

  if(argc < 2) {
    fprintf(stderr, "Please provide a number of obstacles\n");
    return EXIT_FAILURE;
  }

  const size_t N_POLYGONS = atol(argv[1]);
  srand(1234);
  /* clock_t seed = time(0); */
  /* fprintf(stderr, "Seed: %lu\n", seed); */
  /* srand(seed); */

  DList *spheres = dlist_init(), *polygons = dlist_init();
  UNUSED(spheres);

  MapPoint *start, *goal;
  start = malloc(sizeof(MapPoint));
  start->x = 50;
  start->y = 50;

  goal = malloc(sizeof(MapPoint));
  goal->x = -50;
  goal->y = -50;

  random_polygons(polygons, N_POLYGONS);
  polygons = merge_polygons(polygons);


  clock_t t1_full, t2_full, t1_dyn, t2_dyn;
  AStarPathNode *p_full, *p_dyn;
  Graph *g_full, *g_dyn;

  /* fprintf(stderr, "Full\n"); */
  /* fprintf(stderr, "====\n"); */

  t1_full = clock();
  g_full = vgraph_polygonal_obstacles(start,goal,polygons,euclid_distance,0);
  p_full = astar(g_full, start, goal, euclid_distance, hash);
  t2_full = clock();

  /* fprintf(stderr, "Path length: %g\n", p->total_dist); */
  /* fprintf(stderr, "Time; %luus\n", t2-t1); */
  /*  */
  /* fprintf(stderr, "\n"); */
  /* fprintf(stderr, "Dynamic\n"); */
  /* fprintf(stderr, "=======\n"); */

  t1_dyn = clock();
  g_dyn = vgraph_polygonal_obstacles(start,goal,polygons,euclid_distance,2);
  p_dyn = astar(g_dyn, start, goal, euclid_distance, hash);
  t2_dyn = clock();

  /* fprintf(stderr, "Path length: %g\n", p->total_dist); */
  /* fprintf(stderr, "Time; %luus\n", t2-t1); */

  /* printf("Full path:\n"); */
  /* while(p) { */
  /*   print_graph_node(p->data); */
  /*   printf("\n"); */
  /*   p = p->parent; */
  /* } */


  if(p_full->total_dist == p_dyn->total_dist) {
    printf("%lu %lu %lu\n", polygons->num_items, t2_full - t1_full, t2_dyn - t1_dyn);
  } else {
    /* fprintf(stderr, "%g %g\n", p_full->total_dist, p_dyn->total_dist); */
    printf("FAIL\n");
  }


  // The above method may fail. Now let's try the improved method
  t1_dyn = clock();
  g_dyn = vgraph(start,goal,polygons,spheres,euclid_distance,2);
  t2_dyn = clock();

  fprintf(stderr, "Dynamic method 2.0: %lu\n", t2_dyn - t1_dyn);

  dlist_destroy(polygons,NULL);
  polygons = dlist_init();
  evil_test_set(polygons);
  dump_polygons(polygons);
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  graph_print(g_dyn, print_graph_node);
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

  return 0;
}
