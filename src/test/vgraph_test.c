#include "vgraph.h"
#include "astar.h"
#include "tangents.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#define PRINT_FULL_GRAPH 1

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

int main(int argc, const char** argv) {

  // First we need to advertise the start and end location
  // for our path planning instance
  MapPoint *start = malloc(sizeof(MapPoint));
  start->x = 14;
  start->y = 8;

  MapPoint *goal = malloc(sizeof(MapPoint));
  goal->x = -6;
  goal->y = -8;


  // Next we create a list of round obstacles that
  // the agent should move around
  DList *obstacles = dlist_init();
  CircularObstacle *c;

  c = obstacle_init(10,3,2);
  dlist_push(obstacles, c);

  c = obstacle_init(3,0,2);
  dlist_push(obstacles, c);

  c = obstacle_init(-3,-5,2);
  dlist_push(obstacles, c);


  // We want to measure the execution time too
  clock_t t1, t2;

  // This is where the actual magic happens. In this case we use the
  // euclidian distance both to connect the vertices in the graph and
  // to estimate the remaining distance to the goal.
  t1 = clock();
  Graph *g = vgraph_circular_obstacles(start, goal, obstacles, euclid_distance);
  AStarPathNode *p = astar(g, start, goal, euclid_distance, hash);
  t2 = clock();

  // Calculate the time taken in seconds
  double time_taken = ((double) t2 - t1) / CLOCKS_PER_SEC;

  // Output the final path by traversing the list that was returned
  printf("Final path:\n");
  while(p) {
    print_graph_node(p->data);
    printf("\n");
    p = p->parent;
  }

  // And some statistics
  fprintf(stderr, "Time to find it: %gs (%luus)\n", time_taken, t2-t1);


#if PRINT_FULL_GRAPH
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
  printf("(%g,%g)", m->x, m->y);
}
