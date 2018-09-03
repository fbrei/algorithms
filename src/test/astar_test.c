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

void print_node(AStarPathNode *a) {

  MapNode *m = (MapNode*) a->data;

  printf("%5.2lf %5.2lf [%5.2lf]", m->x, m->y, a->estimate + a->total_dist);

}

int main(int argc, const char** argv) {

  MapNode start = { .x = 3, .y = 5 };
  MapNode m1 = { .x = 5, .y = 4 };
  MapNode m2 = { .x = 5, .y = 6 };
  MapNode goal = { .x = 10, .y = 5 };

  Graph *g = graph_init(GRAPH_DIRECTED);

  graph_add(g, &start);
  graph_add(g, &m1);
  graph_add(g, &m2);
  graph_add(g, &goal);

  graph_connect(g, &start, &m1, heuristic(&start,&m1));
  graph_connect(g, &start, &m2, heuristic(&start,&m2));

  graph_connect(g, &m1, &m2, heuristic(&m1,&m2));

  graph_connect(g, &m1, &goal, heuristic(&m1,&goal));
  graph_connect(g, &m2, &goal, heuristic(&m2,&goal));

  AStarPathNode *a = astar(g,&start,&goal,heuristic,NULL);
  AStarPathNode *path = a;
  
  do {
    print_node(a);
    printf("\n");
    a = a->parent;
  } while (a != NULL);

  astar_free_path(path);
  graph_destroy(g,NULL);

  return 0;
}
