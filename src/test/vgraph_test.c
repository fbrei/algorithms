#include "vgraph.h"
#include "astar.h"
#include "tangents.h"

#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

/**
 * Calculates the euclidian distance between two map points
 * that must be given as void pointers for compatibility
 */
double euclid_distance(void*, void*);

unsigned long hash(void *, void*);

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

  c = obstacle_init(3,2,2);
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
  AStarPathNode *p = astar(g, start, goal, euclid_distance, NULL);
  t2 = clock();

  // Calculate the time taken in seconds
  double time_taken = ((double) t2 - t1) / CLOCKS_PER_SEC;

  // Output the final path by traversing the list that was returned
  printf("Final path:\n");
  while(p) {
    MapPoint *m = p->data;
    printf("(%g,%g)\n",m->x, m->y);
    p = p->parent;
  }

  // And some statistics
  printf("Time to find it: %gs\n", time_taken);

  return EXIT_SUCCESS;
}



double euclid_distance(void *n1, void *n2) {

  MapPoint *m1 = (MapPoint*) n1;
  MapPoint *m2 = (MapPoint*) n2;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);
}


unsigned long hash(void *n1, void *n2) {
  MapPoint *m1 = (MapPoint*) n1;
  MapPoint *m2 = (MapPoint*) n2;

  union longdouble {
    unsigned long out;
    double in;
  } a,b,c,d ;

  a.in = m1->x;
  b.in = m1->y;
  c.in = m2->x;
  d.in = m2->y;

  return (a.out & 0xFF) | ((b.out << 8) & 0xFF00) | ((c.out << 16) & 0xFF0000) | ((d.out << 24) & 0xFF000000);
}
