#include "include/vgraph.h"
#include "include/astar.h"
#include "include/tangents.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define PRINT_FULL_GRAPH 0
#define TEST_RANDOM 1
#define RANDOM_OBST 20
// Test sets:
// 1 - 
// 2 - 
// 3 - 
// 4 - Dynamic fails because visibility cone in front of
//     obstale was not taken account of
// 5 - After fixing 4, Dynamic fails because the start is
//     not being connected to any obstacle at all
#define TEST_SET 5
#define BOTH_METHODS 0
#define MEASUREMENT_OUTPUT 1

/**
 * Calculates the euclidian distance between two map points
 * that must be given as void pointers for compatibility
 */
double euclid_distance(void*, void*);


/**
 * Can be used to print out the graph for postprocessing
 */
void print_graph_node(void*);

unsigned long hash(void *);
int randint(int min, int max) {
  return (int) (( (double) rand() ) / RAND_MAX * (max-min) + min);
}

void add_random_obstacle(DList *obstacles, DList *work_obstacles, MapPoint *goal, MapPoint *start) {

  unsigned int num_tries = 0;
  while(num_tries < 50) {
    CircularObstacle *c = obstacle_init(randint(-48,48),randint(-48,48),6);

    CircularObstacle *o = NULL;

    int non_overlapping = 1;
    if(euclid_distance(&(c->position), start) <= 6) {
      non_overlapping = 0;
    }
    if(euclid_distance(&(c->position), goal) <= 6) {
      non_overlapping = 0;
    }
    while((o = dlist_iterate(obstacles,o)) != NULL) {
      if(euclid_distance(&(c->position), &(o->position)) <= 12) {
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

int main() {

  srand(time(0));

  // First we need to advertise the start and end location
  // for our path planning instance
  MapPoint *start1 = malloc(sizeof(MapPoint));
  start1->x = 50;
  start1->y = 50;

  MapPoint *goal1 = malloc(sizeof(MapPoint));
  goal1->x = -50;
  goal1->y = -50;

  MapPoint *start2 = malloc(sizeof(MapPoint));
  start2->x = 50;
  start2->y = 50;

  MapPoint *goal2 = malloc(sizeof(MapPoint));
  goal2->x = -50;
  goal2->y = -50;

  // Next we create a list of round obstacles that
  // the agent should move around
  DList *obstacles = dlist_init(), *work_obstacles = dlist_init();

#if TEST_RANDOM == 1
  for(size_t ii = 0; ii < RANDOM_OBST; ii++) {
    add_random_obstacle(obstacles, work_obstacles, start1, goal1);
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
#endif

#endif

  // We want to measure the execution time too
  clock_t t1, t2, t_inter;

  Graph *g1, *g2;
  AStarPathNode *p1, *p2, *p1_start, *p2_start;

  // This is where the actual magic happens. In this case we use the
  // euclidian distance both to connect the vertices in the graph and
  // to estimate the remaining distance to the goal.
  
  // We also compare different graph building strategies
#if BOTH_METHODS == 1
  t1 = clock();
  g1 = vgraph_circular_obstacles(start1, goal1, work_obstacles, euclid_distance,0);
  t_inter = clock();
  p1 = astar(g1, start1, goal1, euclid_distance, hash);
  t2 = clock();
  p1_start = p1;
  fprintf(stderr,"Full graph: %lu | %lu | %g\n", t2 - t_inter, t_inter - t1, p1->total_dist);

  CircularObstacle *tmp_obst = NULL;
  while((tmp_obst = (CircularObstacle*) dlist_iterate(work_obstacles,tmp_obst)) != NULL) {
    darray_destroy(tmp_obst->_map_points,NULL);
    tmp_obst->_map_points = darray_init();
    tmp_obst->_num_map_points = 0;
  }
#endif

  t1 = clock();
  g2 = vgraph_circular_obstacles(start2, goal2, work_obstacles, euclid_distance,0);
  t_inter = clock();
  p2 = astar(g2, start2, goal2, euclid_distance, hash);
  t2 = clock();
  p2_start = p2;

#if MEASUREMENT_OUTPUT
  if(p2) {
    fprintf(stderr,"%lu\n", t2 - t1);
  } else {
    fprintf(stderr,"%lu\n", 999999l);
  }
#else
  if(p2) {
    fprintf(stderr,"Dynamic: %lu | %lu | %g\n", t2 - t_inter, t_inter - t1, p2->total_dist);
  } else {
    fprintf(stderr,"Dynamic: %lu | %lu | %g\n", t2 - t_inter, t_inter - t1, 99999.9);
  }
#endif

  // Calculate the time taken in seconds

  /* if(p != NULL) { */
  /*   fprintf(stderr, "%lu\n",tdiff); */
  /* } else { */
  /*   fprintf(stderr,"999999\n"); */
  /* } */

  // And some statistics
#if PRINT_FULL_GRAPH
  /*  */
  /* // Output the final path by traversing the list that was returned */
  /* printf("Final path:\n"); */
  /* while(p1) { */
  /*   print_graph_node(p1->data); */
  /*   printf("\n"); */
  /*   p1 = p1->parent; */
  /* } */
  /*  */
  /*  */
  /* // This will be used by a python script to create a plot */
  /* printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"); */
  /* graph_print(g1, print_graph_node); */
  /* printf("OBSTACLES: "); */
  /* while((obst = dlist_iterate(obstacles,obst)) != NULL) { */
  /*   CircularObstacle *c = (CircularObstacle*) obst; */
  /*   printf("(%g,%g,%g),",c->position.x, c->position.y, c->radius); */
  /* } */
  /* printf("\n"); */
  /* printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"); */
#endif


#if PRINT_FULL_GRAPH
  // Output the final path by traversing the list that was returned
  printf("Final path:\n");
  while(p2) {
    print_graph_node(p2->data);
    printf("\n");
    p2 = p2->parent;
  }


  // This will be used by a python script to create a plot
  CircularObstacle *obst = NULL;
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  graph_print(g2, print_graph_node);
  printf("OBSTACLES: ");
  while((obst = (CircularObstacle*) dlist_iterate(obstacles,obst)) != NULL) {
    CircularObstacle *c = (CircularObstacle*) obst;
    printf("(%g,%g,%g),",c->position.x, c->position.y, c->radius);
  }
  printf("\n");
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
#endif

  dlist_destroy(obstacles,obstacle_destroy);
  dlist_destroy(work_obstacles,NULL);

#if BOTH_METHODS == 1
  graph_destroy(g1,free);
  astar_free_path(p1_start);
#endif

  graph_destroy(g2,free);
  astar_free_path(p2_start);

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
