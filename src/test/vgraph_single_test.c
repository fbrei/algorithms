#define SFMT_MEXP 19937

#include "include/vgraph.h"
#include "include/astar.h"
#include "include/tangents.h"
#include "lib/sfmt/SFMT.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define PRINT_FULL_GRAPH 0
#define TEST_RANDOM 1
// Test sets:
// -1 . random
// 1 - 
// 2 - 
// 3 - 
// 4 - Dynamic fails because visibility cone in front of
//     obstale was not taken account of
// 5 - After fixing 4, Dynamic fails because the start is
//     not being connected to any obstacle at all
// 6 - First test set with overlapping obstacles (U shape)
// 7 - First test set with overlapping obstacles (V shape)
// 8 - Dynamic takes more time than full (A*)
#define TEST_SET 8

#define INDIVIDUAL_RUNS 2

#ifndef M_PI
#define M_PI 3.1415926535897932
#endif


/**
 * Calculates the euclidian distance between two map points
 * that must be given as void pointers for compatibility
 */
double euclid_distance(void*, void*);

/**
 * Initializes the obstacle lists
 * testcase - number of case, or -1 for random
 */
void init_obstacles(DList *obstacles, DList *work_obstacles, int testcase, MapPoint* start, MapPoint *goal);

/**
 * Reset obstacles after each run.
 */
void reset_obstacles(DList *obstacles);

/**
 * Make a test run using a specified method (0 full, 2 dynamic)
 */
unsigned long test_run(DList* obstacles, int method);

static sfmt_t state;
static unsigned int RANDOM_OBST;
static double OBST_RADIUS;

/**
 * Can be used to print out the graph for postprocessing
 */
void print_graph_node(void*);

unsigned long hash(void *);

int randint(int min, int max) {
  return (sfmt_genrand_uint32(&state) % (max - min)) + min;
}

void add_random_obstacle(DList *obstacles, DList *work_obstacles, MapPoint *goal, MapPoint *start) {

  unsigned int num_tries = 0;
  while(num_tries < 50) {
    CircularObstacle *c = obstacle_init(randint(-48,48),randint(-48,48),OBST_RADIUS);

    int non_overlapping = 1;
    if(euclid_distance(&(c->position), start) <= OBST_RADIUS) {
      non_overlapping = 0;
    }
    if(euclid_distance(&(c->position), goal) <= OBST_RADIUS) {
      non_overlapping = 0;
    }

    CircularObstacle *o = NULL;

    while((o = dlist_iterate(obstacles,o)) != NULL) {
      if(euclid_distance(&(c->position), &(o->position)) <= 2 * OBST_RADIUS) {
      /* if(euclid_distance(&(c->position), &(o->position)) <= 0.1 ) { */
        non_overlapping = 0;
        break;
      }
    }

    if(non_overlapping) {
      dlist_push(obstacles, c);
      dlist_push(work_obstacles, c);
      return;
    } else {
      num_tries++;
      obstacle_destroy(c);
    }
  }
}

int main(int argc, const char** argv) {

  /* sfmt_init_gen_rand(&state,1991); */
  sfmt_init_gen_rand(&state,1991);

  // First we need to advertise the start and end location
  // for our path planning instance
  MapPoint *start1 = malloc(sizeof(MapPoint));
  start1->x = 50;
  start1->y = 50;

  MapPoint *goal1 = malloc(sizeof(MapPoint));
  goal1->x = -50;
  goal1->y = -50;

  RANDOM_OBST = atoi(argv[1]);
  OBST_RADIUS = 6.0;
  OBST_RADIUS = sqrt( (10000 * 0.2261946710584651104) / (RANDOM_OBST * M_PI) );

  unsigned long t1 = 0, t2 = 0;

  fprintf(stderr, "Total | VG | A* | L\n");
  for(int ii = 0; ii < 1; ii++) {
    DList *obstacles = dlist_init(), *work_obstacles = dlist_init();
    init_obstacles(obstacles, work_obstacles, TEST_SET, start1, goal1);
    fprintf(stderr, "%d | %d\n",RANDOM_OBST, ii);

    clock_t t1 = test_run(work_obstacles,0);
    reset_obstacles(work_obstacles);

    clock_t t2 = test_run(work_obstacles,2);
    reset_obstacles(work_obstacles);

    printf("\n");
    dlist_destroy(obstacles,obstacle_destroy);
    dlist_destroy(work_obstacles,NULL);
  }

  return EXIT_SUCCESS;
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

void print_graph_node(void* v) {

  MapPoint *m = (MapPoint*) v;
  /* printf("(%g,%g) | %g", m->x, m->y,m->score); */
  printf("(%g,%g)", m->x, m->y);
}

void reset_obstacles(DList *obstacles) {
  CircularObstacle *tmp_obst = NULL;
  while((tmp_obst = (CircularObstacle*) dlist_iterate(obstacles,tmp_obst)) != NULL) {
    darray_destroy(tmp_obst->_map_points,NULL);
    tmp_obst->_map_points = darray_init();
    tmp_obst->_num_map_points = 0;
  }
}

unsigned long test_run(DList* obstacles, int method) {

  MapPoint *start = malloc(sizeof(MapPoint));
  start->x = 50;
  start->y = 50;

  MapPoint *goal = malloc(sizeof(MapPoint));
  goal->x = -50;
  goal->y = -50;

  clock_t t1, t2, t_inter;
  Graph *g;
  AStarPathNode *p;

  t1 = clock();
  g = vgraph_circular_obstacles(start, goal, obstacles, euclid_distance,method);
  t_inter = clock();
  p = astar(g, start, goal, euclid_distance, hash);
  t2 = clock();

#if PRINT_FULL_GRAPH

  AStarPathNode *p_start = p;

  // Output the final path by traversing the list that was returned
  printf("Final path:\n");
  while(p) {
    print_graph_node(p->data);
    printf("\n");
    p = p->parent;
  }


  // This will be used by a python script to create a plot
  CircularObstacle *obst = NULL;
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  graph_print(g, print_graph_node);
  printf("OBSTACLES: ");
  while((obst = (CircularObstacle*) dlist_iterate(obstacles,obst)) != NULL) {
    CircularObstacle *c = (CircularObstacle*) obst;
    printf("(%g,%g,%g),",c->position.x, c->position.y, c->radius);
  }
  printf("\n");
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

  p = p_start;
#endif

  fprintf(stderr, "%lu %lu %lu ", t2 - t1, t_inter - t1, t2 - t_inter);
  fprintf(stderr, "%g\n", p->total_dist);
  /* printf("%lu ", t2-t1); */
  graph_destroy(g,free);
  astar_free_path(p);

  return t2 - t1;
}

void init_obstacles(DList *obstacles, DList *work_obstacles, int testcase, MapPoint* start, MapPoint *goal) {

#if TEST_RANDOM == 1
  for(size_t ii = 0; ii < RANDOM_OBST; ii++) {
    add_random_obstacle(obstacles, work_obstacles, start, goal);
  }
#else
  CircularObstacle *c;
#if TEST_SET == 1
  c = obstacle_init(-10,-12,9);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(22,22,9);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(29,-10,9);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(25,44,9);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-37,4,9);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(7,-32,9);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
#elif TEST_SET == 2
  c = obstacle_init(-22,-15,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(0,-12,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(-5,-33,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(30,32,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(-18,-38,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(13,26,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(-34,-24,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(28,44,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(-17,-3,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(-10,-21,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(-42,-47,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

#elif TEST_SET == 3

  c = obstacle_init(-10,-10,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(10,20,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

  c = obstacle_init(20,10,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles,c);

#elif TEST_SET == 4
  c = obstacle_init(-39,10,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-5,21,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(28,20,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(16,-30,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-12,0,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(30,-32,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-22,-10,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(7,-10,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-31,26,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(11,8,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(34,-46,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(33,-8,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-2,38,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(18,33,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-25,-27,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(10,44,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-8,-28,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-15,-45,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(33,6,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(45,-40,6);
  dlist_push(obstacles,c);
  dlist_push(work_obstacles, c);

#elif TEST_SET == 5
  c = obstacle_init(-28,-21,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-21,28,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(46,0,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(22,13,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(6,-33,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(29,28,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(16,27,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(33,-40,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-8,0,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-9,-44,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(4,35,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-33,9,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-9,24,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(14,-7,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(40,15,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-19,12,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(-40,43,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(41,34,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(2,8,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
  
  c = obstacle_init(18,-47,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
#elif TEST_SET == 6
  c = obstacle_init(-25,-25,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-31,-17,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-36,-9,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-40,-1,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-42,7,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-17,-31,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-9,-36,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-1,-40,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(7,-42,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
#elif TEST_SET == 7
  c = obstacle_init(0,0,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(2,-11,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-11,2,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
#elif TEST_SET == 8
  c = obstacle_init(-6,-5,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-26,13,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(35,-23,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(28,32,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-31,41,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-29,-8,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(9,17,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-10,-44,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-2,9,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(23,4,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(24,-41,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(9,36,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-27,-44,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(45,5,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-18,46,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(39,27,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-5,44,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(10,5,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(31,-10,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);

  c = obstacle_init(-46,-25,6);
  dlist_push(obstacles, c);
  dlist_push(work_obstacles, c);
#endif
#endif
}
