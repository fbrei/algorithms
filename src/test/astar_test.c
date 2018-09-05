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

void example1() {

  Graph *g = graph_init(GRAPH_DIRECTED);

  MapNode start = { .x = 5, .y = 4 };
  MapNode goal = { .x = 1, .y = 2 };

  graph_add(g, &start);
  graph_add(g, &goal);

  MapNode m1 = { .x = 4, .y = 4 };
  MapNode m2 = { .x = 4, .y = 5 };
  graph_add(g,&m1);
  graph_add(g,&m2);
  graph_connect(g, &start, &m1, heuristic(&start, &m1));
  graph_connect(g, &start, &m2, heuristic(&start, &m2));

  MapNode m3 = { .x = 3, .y = 3 };
  MapNode m4 = { .x = 3, .y = 1 };
  graph_add(g,&m3);
  graph_add(g,&m4);
  graph_connect(g, &start, &m3, heuristic(&start, &m3));
  graph_connect(g, &start, &m4, heuristic(&start, &m4));

  graph_connect(g, &m1, &m3, heuristic(&m1, &m3));

  MapNode m5 = { .x = 3, .y = 5 };
  MapNode m6 = { .x = 3, .y = 4 };
  graph_add(g,&m5);
  graph_add(g,&m6);
  graph_connect(g, &m2, &m5, heuristic(&m2, &m5));
  graph_connect(g, &m1, &m6, heuristic(&m1, &m6));

  MapNode m7 = { .x = 2, .y = 3 };
  MapNode m8 = { .x = 2, .y = 1 };
  graph_add(g,&m7);
  graph_add(g,&m8);
  graph_connect(g, &m3, &m7, heuristic(&m3, &m7));
  graph_connect(g, &m4, &m8, heuristic(&m4, &m8));


  graph_connect(g, &m6, &m7, heuristic(&m6, &m7));
  graph_connect(g, &m1, &m7, heuristic(&m1, &m7));
  graph_connect(g, &m5, &m7, heuristic(&m5, &m7));

  graph_connect(g, &m5, &goal, heuristic(&m5, &goal));
  graph_connect(g, &m7, &goal, heuristic(&m7, &goal));
  graph_connect(g, &m8, &goal, heuristic(&m8, &goal));

  AStarPathNode *a = astar(g,&start,&goal,heuristic,NULL);
  AStarPathNode *path = a;
  
  do {
    print_node(a);
    printf("\n");
    a = a->parent;
  } while (a != NULL);

  astar_free_path(path);
  graph_destroy(g,NULL);

}

void example2() {

  

}

int main(int argc, const char** argv) {

  example1();

  return 0;
}
