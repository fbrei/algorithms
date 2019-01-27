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

#define RUN_FULL 1
#define RUN_DYNAMIC 1

#define min(x,y) ((x) < (y)) ? (x) : (y)

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

double path_length_vgraph(MapPoint *m) {


  double total_length = 0.0;

  while(m != NULL) {
    if(m->shortest_ancestor != NULL) {
      total_length += euclid_distance(m, m->shortest_ancestor);
    }
    m = m->shortest_ancestor;
  }

  return total_length;
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
  m->shortest_length = 9999999999.9l;

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

void random_polygons(DList *polygons, const size_t N_POLYGONS, const double x_dim, const double y_threshold) {

  const size_t MIN_CORNERS = 4;
  const size_t MAX_CORNERS = 9;

  const double WIDTH = 2 * x_dim;
  const double MIN_COORD = -x_dim;
  const double SPREAD = 120 / N_POLYGONS;

  for(size_t ii = 0; ii < N_POLYGONS; ii++) {
    DList *base_points = dlist_init();
    MapPoint *m = malloc(sizeof(MapPoint));
    m->x = (((double) rand()) / RAND_MAX) * WIDTH + MIN_COORD;

    double dec = y_threshold / x_dim;
    double y_dim = min(dec + y_threshold, -dec / y_threshold);
    m->y = (((double) rand()) / RAND_MAX) * 2.0 * y_dim - y_dim + m->x;
    m->shortest_length = 9999999999.9l;

    dlist_push(base_points, m);
    size_t n_corners = rand() % (MAX_CORNERS - MIN_CORNERS) + MIN_CORNERS;
    for(size_t jj = 0; jj < n_corners; jj++) {
      MapPoint *n = malloc(sizeof(MapPoint));
      n->x = ((double) rand()) / RAND_MAX * 2 * SPREAD - SPREAD + m->x;
      n->y = ((double) rand()) / RAND_MAX * 2 * SPREAD - SPREAD + m->y;
      dlist_push(base_points, n);
      n->shortest_length = 9999999999.9l;
    }

    dlist_push(polygons, convex_hull(base_points));
  }

}


void evil_test_set(DList *polygons) {

  DList *points;
  MapPoint *m;
  PolygonalObstacle *p;

  // =========================================================

  p = malloc(sizeof(PolygonalObstacle));
  points = dlist_init();

  m = malloc(sizeof(MapPoint));
  m->x = 10;
  m->y = 30;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = 30;
  m->y = 10;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = 10;
  m->y = -10;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = -10;
  m->y = 10;
  m->obstacle = p;
  dlist_push(points,m);

  p->corners = points;
  dlist_push(polygons,p);

  // =========================================================

  p = malloc(sizeof(PolygonalObstacle));
  points = dlist_init();

  m = malloc(sizeof(MapPoint));
  m->x = -21;
  m->y = 16;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = -4;
  m->y = -1;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = -10;
  m->y = -7;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = -27;
  m->y = 10;
  m->obstacle = p;
  dlist_push(points,m);

  p->corners = points;
  dlist_push(polygons,p);

  // =========================================================

  p = malloc(sizeof(PolygonalObstacle));
  points = dlist_init();

  m = malloc(sizeof(MapPoint));
  m->x = -1;
  m->y = -4;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = 30;
  m->y = -35;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = 10;
  m->y = -55;
  m->obstacle = p;
  dlist_push(points,m);

  m = malloc(sizeof(MapPoint));
  m->x = -21;
  m->y = -24;
  m->obstacle = p;
  dlist_push(points,m);

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
      for(size_t jj = ii+1; jj < new_polygons->num_items; jj++) {
        PolygonalObstacle *o = darray_get(new_polygons->data, jj);

        if(polygon_overlapping(p, o)) {
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

  for(size_t ii = 0; ii < new_polygons->num_items; ii++) {
    PolygonalObstacle *p = darray_get(new_polygons->data, ii);
    for(size_t jj = 0; jj < p->corners->num_items; jj++) {
      MapPoint *m = darray_get(p->corners->data, jj);
      m->obstacle = p;
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

void save_polygons(DList *polygons, size_t seed) {

  const size_t BUFFER_SIZE = 256;

  char *fname = malloc(BUFFER_SIZE * sizeof(char));
  snprintf(fname, BUFFER_SIZE, "%lu_%lu_%lu.bin", polygons->num_items, seed, (unsigned long) round(100.0 * polygon_map_covered(polygons, 100, 100)));
  fprintf(stderr, "Saving as %s\n", fname);

}

// =========================================================

void print_help() {
  printf("Usage: bin/vgraph_total_test [-n number of obstacles] [-s seed] [-o outfile] [-v] [-h] [-d]\n");
  printf("    -n : The number of obstacles that shall be randomly generated. \n");
  printf("         Note that the real number will be a bit lower due to merging\n");
  printf("         (defaults to 20).\n");
  printf("    -s : The seed value for the RNG. Default is time(0).\n");
  printf("    -o : File name that the graph should be written to.\n");
  printf("    -v : Display some more output for the dynamic method. Subject to change.\n");
  printf("    -h : Display this help.\n");
  printf("    -d : Dump the polygons in a json like array (can be read by python for example).\n");
}

void _reset_point(MapPoint *m) {
  m->is_in = 0;
  dlist_destroy(m->origins, NULL);
  m->obstacle = NULL;
  dlist_destroy(m->visited_points, NULL);
  m->shortest_length = 9999999999.9l;
  m->shortest_ancestor = NULL;
  dlist_destroy(m->reachable, NULL);
}
void reset_map_points(MapPoint *start, MapPoint *goal, DList *polygons) {

  _reset_point(start);
  _reset_point(goal);

  for(size_t idx_p = 0; idx_p < polygons->num_items; idx_p++) {
    PolygonalObstacle *p = darray_get(polygons->data,idx_p);
    for(size_t idx_m = 0; idx_m < p->corners->num_items; idx_m++) {
      MapPoint *m = darray_get(p->corners->data, idx_m);
      _reset_point(m);
    }
  }
}

int main(int argc, char** argv) {

  size_t N_POLYGONS = 20;
  short VERBOSE = 0;
  clock_t seed = time(0);
  srand(seed);
  seed = rand();

  // Use getopt to parse the arguments from the command line
  extern char* optarg;
  extern int optind;
  short n_set = 0, h_set = 0, o_set = 0, d_set = 0;
  char* outfile_name = NULL;
  short rc = 0, err = 0;

  while((rc = getopt(argc, argv, "dvhn:s:o:")) != -1) {
    switch(rc) {
      case 'v':
        VERBOSE++;
        break;
      case 'h':
        h_set = 1;
        break;
      case 'd':
        d_set = 1;
        break;
      case 'n':
        n_set = 1;
        N_POLYGONS = atol(optarg);
        break;
      case 's':
        seed = atol(optarg);
        break;
      case 'o':
        o_set = 1;
        size_t slen = strlen(optarg);
        outfile_name = malloc((slen+1) * sizeof(char));
        memcpy(outfile_name, optarg, slen);
        outfile_name[slen] = 0x0;
        break;
      default:
        err = 1;
    }
  }

  if(err == 1 || h_set == 1) {
    print_help();
    return err;
  }

  if(n_set == 0) {
    N_POLYGONS = 20;
  }

  // Example I chose for the comparison plot
  /* clock_t seed = 1546981252; */

  // Test for correctness
  /* clock_t seed = 1547149155; */

  // Default

  srand(seed);

  DList *spheres = dlist_init(), *polygons = dlist_init();
  UNUSED(spheres);

  MapPoint *start, *goal;
  start = malloc(sizeof(MapPoint));
  start->x = 50;
  start->y = 50;
  start->obstacle = NULL;

  goal = malloc(sizeof(MapPoint));
  goal->x = -50;
  goal->y = -50;
  goal->obstacle = NULL;
  goal->shortest_length = 9999999999.9l;

  clock_t t_aou6ff0n, t_khlrengx;
  
  t_aou6ff0n = clock();
  random_polygons(polygons, N_POLYGONS, 45.0, 20);
  polygons = merge_polygons(polygons);
  t_khlrengx = clock();
  
  clock_t map_time = t_khlrengx - t_aou6ff0n;

  /* evil_test_set(polygons); */

  clock_t t1_full, t2_full;
  AStarPathNode *p_full = NULL;
  Graph *g_full = NULL; 

  t1_full = clock();
  g_full = vgraph_polygonal_obstacles(start,goal,polygons,euclid_distance,0);
  p_full = astar(g_full, start, goal, euclid_distance, hash);
  t2_full = clock();

  /* fprintf(stderr, "\n"); */

  /* t1_dyn = clock(); */
  /* g_dyn = vgraph_polygonal_obstacles(start,goal,polygons,euclid_distance,2); */
  /* p_dyn = astar(g_dyn, start, goal, euclid_distance, hash); */
  /* t2_dyn = clock(); */

  /* fprintf(stderr, "Path length: %g\n", p->total_dist); */
  /* fprintf(stderr, "Time; %luus\n", t2-t1); */


  /* if(p_full->total_dist == p_dyn->total_dist) { */
  /*   printf("%lu %lu %lu\n", polygons->num_items, t2_full - t1_full, t2_dyn - t1_dyn); */
  /* } else { */
  /*   [> fprintf(stderr, "%g %g\n", p_full->total_dist, p_dyn->total_dist); <] */
  /*   printf("FAIL\n"); */
  /* } */

  /* fprintf(stderr, "Dynamic method 2.0: %lu\n", t2_dyn - t1_dyn); */

  /* dlist_destroy(polygons,NULL); */
  /* polygons = dlist_init(); */

  if(d_set) dump_polygons(polygons);

  // The above method may fail. Now let's try the improved method
  clock_t t1_dyn, t2_dyn;
  AStarPathNode *p_dyn = NULL;
  Graph *g_dyn = NULL; 

  t1_dyn = clock();
  g_dyn = vgraph(start,goal,polygons,spheres,euclid_distance,VGRAPH_DYNAMIC, VERBOSE);
  p_dyn = astar(g_dyn, start, goal, euclid_distance, hash);
  t2_dyn = clock();

  clock_t t1_vstar, t2_vstar;
  t1_vstar = clock();
  /* vstar(start, goal, polygons, spheres, euclid_distance, VERBOSE); */
  t2_vstar = clock();

  clock_t t1_dyn_pr, t2_dyn_pr;
  AStarPathNode *p_dyn_pr = NULL;
  Graph *g_dyn_pr = NULL; 

  t1_dyn_pr = clock();
  g_dyn_pr = vgraph(start,goal,polygons,spheres,euclid_distance,VGRAPH_DYNAMIC_LOCAL_PRUNING, VERBOSE);
  p_dyn_pr = astar(g_dyn_pr, start, goal, euclid_distance, hash);
  t2_dyn_pr = clock();
  
  if(VERBOSE) {
    printf("====================================\n");
    printf("Meta information:\n");
    printf("----------------------\n");
    printf("   Seed: %lu\n", seed);
    printf("   Time to generate map: %luus\n", map_time);
    printf("   Number of obstacles: %lu\n", polygons->num_items);
    printf("   Percentage of map covered: %g\n", polygon_map_covered(polygons, 100, 100));
    printf("====================================\n");
    printf("\n");

    printf("Full\n");
    printf("====================================\n");

    printf("Path length: %g\n", p_full->total_dist);
    printf("Time; %luus\n", t2_full-t1_full);

    printf("\n");

    printf("Dynamic\n");
    printf("====================================\n");

    printf("Path length: %g\n", p_dyn->total_dist);
    printf("Time; %luus\n", t2_dyn-t1_dyn);

    printf("\n");

    printf("Dynamic with pruning\n");
    printf("====================================\n");

    printf("Path length: %g\n", p_dyn_pr->total_dist);
    printf("Time; %luus\n", t2_dyn_pr-t1_dyn_pr);

    printf("\n");

    printf("VStar\n");
    printf("====================================\n");

    printf("Path length: %g\n", 0.0);
    printf("Time; %luus\n", t2_vstar - t1_vstar);
  } else {
    printf("%lu %lu %lu %lu %lu\n", polygons->num_items, seed, t2_full - t1_full, t2_dyn - t1_dyn, t2_dyn_pr - t1_dyn_pr);
  }

#define PRINTED_GRAPH g_dyn
#define PRINTED_PATH p_dyn
  if(o_set) {
    freopen(outfile_name, "w", stdout);
    printf("Full path:\n");
    while(PRINTED_PATH) {
      print_graph_node(PRINTED_PATH->data);
      printf("\n");
      PRINTED_PATH = PRINTED_PATH->parent;
    }
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
    graph_print(PRINTED_GRAPH, print_graph_node);
    printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  }

  /* save_polygons(polygons, seed); */
  UNUSED(p_dyn);
  UNUSED(p_full);

  return 0;
}
