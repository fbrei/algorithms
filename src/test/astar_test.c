#include "astar.h"
#include "dtype.h"
#include <math.h>
#include <stdio.h>

typedef struct _MapNode {
  double x;
  double y;
} MapNode;


double heuristic(void *n1, void *n2)  {

  MapNode *m1 = (MapNode*) n1;
  MapNode *m2 = (MapNode*) n2;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);

}

int equals(void *first, void *second) {

  MapNode *m1 = (MapNode*) first;
  MapNode *m2 = (MapNode*) second;

  return (m1->x == m2->x && m1->y == m2->y) ? 1 : 0;

}


void print_node(AStarPathNode *a) {

  MapNode *m = (MapNode*) a->data;

  printf("%5.2lf %5.2lf ", m->x, m->y);

}

int main(int argc, const char** argv) {

  MapNode start = { .x = 1, .y = 2 };
  MapNode m1 = { .x = 3, .y = 3 };
  MapNode goal = { .x = 10, .y = 4 };

  Graph *g = graph_init(GRAPH_DIRECTED);

  graph_add(g, &start);
  graph_add(g, &m1);
  graph_add(g, &goal);

  graph_connect(g, &start, &m1, heuristic(&start,&m1));
  graph_connect(g, &m1, &goal, heuristic(&m1,&goal));

  AStarPathNode *a = astar(g,&start,&goal,heuristic,equals);
  
  do {
    print_node(a);
    printf("\n");
    a = a->parent;
  } while (a != NULL);

  graph_destroy(g,NULL);

  return 0;
}
