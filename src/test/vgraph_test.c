#include "include/vgraph.h"
#include "include/astar.h"
#include "include/tangents.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define PRINT_FULL_GRAPH 1
#define TEST_RANDOM 0
#define RANDOM_OBST 20

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
    if(euclid_distance(&(c->position), start) < 6) {
      non_overlapping = 0;
    }
    if(euclid_distance(&(c->position), goal) < 6) {
      non_overlapping = 0;
    }
    while((o = dlist_iterate(obstacles,o)) != NULL) {
      if(euclid_distance(&(c->position), &(o->position)) < 12) {
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
  MapPoint *start = malloc(sizeof(MapPoint));
  start->x = 50;
  start->y = 50;

  MapPoint *goal = malloc(sizeof(MapPoint));
  goal->x = -50;
  goal->y = -50;

  // Next we create a list of round obstacles that
  // the agent should move around
  DList *obstacles = dlist_init(), *work_obstacles = dlist_init();
  CircularObstacle *c;

#if TEST_RANDOM == 1
  for(size_t ii = 0; ii < RANDOM_OBST; ii++) {
    add_random_obstacle(obstacles, work_obstacles, start, goal);
  }
#else
  c = obstacle_init(-20,-15,9);
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
#endif
  // We want to measure the execution time too
  clock_t t1, t2, t_inter;

  // This is where the actual magic happens. In this case we use the
  // euclidian distance both to connect the vertices in the graph and
  // to estimate the remaining distance to the goal.
  t1 = clock();
  Graph *g = vgraph_circular_obstacles(start, goal, work_obstacles, euclid_distance);
  t_inter = clock();
  AStarPathNode *p = astar(g, start, goal, euclid_distance, hash);
  t2 = clock();

  // Calculate the time taken in seconds
  double time_taken = ((double) t2 - t1) / CLOCKS_PER_SEC, time_;
  clock_t tdiff = t2 - t1;

  if(p != NULL) {
    fprintf(stderr, "%lu\n",tdiff);
  } else {
    fprintf(stderr,"999999\n");
  }

  // And some statistics
#if PRINT_FULL_GRAPH
  fprintf(stderr, "Time to find it: %gs (%luus)\n", time_taken, t2-t1);
  fprintf(stderr, "%lu - %lu\n", t_inter - t1, t2 - t_inter);
  if(p != NULL) {
    fprintf(stderr, "Total cost: %g\n", p->total_dist);
  } else {
    fprintf(stderr,"No path found!\n");
  }

  // Output the final path by traversing the list that was returned
  printf("Final path:\n");
  while(p) {
    print_graph_node(p->data);
    printf("\n");
    p = p->parent;
  }


  // This will be used by a python script to create a plot
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
  graph_print(g, print_graph_node);
  void *obst = NULL;
  printf("OBSTACLES: ");
  while((obst = dlist_iterate(obstacles,obst)) != NULL) {
    CircularObstacle *c = (CircularObstacle*) obst;
    printf("(%g,%g,%g),",c->position.x, c->position.y, c->radius);
  }
  printf("\n");
  printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
#endif

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
