#include "include/astar.h"
#include "lib/dtypes/include/dtype.h"
#include <math.h>
#include <stdio.h>
#include <time.h>

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


unsigned long hash(void *item) {

  return (long) item;

}


void example1() {

  Graph *g = graph_init(GRAPH_DIRECTED);

  MapNode *start = malloc(sizeof(MapNode));
  start->x = 5, start->y = 4;

  MapNode *goal = malloc(sizeof(MapNode));
  goal->x= 1, goal->y = 2;

  graph_add(g, start);
  graph_add(g, goal);

  MapNode *m1 = malloc(sizeof(MapNode));
  m1->x = 4, m1->y = 4;
  MapNode *m2 = malloc(sizeof(MapNode));
  m2->x= 4, m2->y = 5;
  graph_add(g,m1);
  graph_add(g,m2);
  graph_connect(g, start, m1, heuristic(start, m1));
  graph_connect(g, start, m2, heuristic(start, m2));

  MapNode *m3 = malloc(sizeof(MapNode));
  m3->x = 3, m3->y = 3;
  MapNode *m4 = malloc(sizeof(MapNode));
  m4->x = 3, m4->y = 1;
  graph_add(g,m3);
  graph_add(g,m4);
  graph_connect(g, start, m3, heuristic(start, m3));
  graph_connect(g, start, m4, heuristic(start, m4));

  graph_connect(g, m1, m3, heuristic(m1, m3));

  MapNode *m5 = malloc(sizeof(MapNode));
  m5->x = 3, m5->y = 5;
  MapNode *m6 = malloc(sizeof(MapNode));
  m6->x = 3, m6->y = 4;

  graph_add(g,m5);
  graph_add(g,m6);

  graph_connect(g, m2, m5, heuristic(m2, m5));
  graph_connect(g, m1, m6, heuristic(m1, m6));

  MapNode *m7 = malloc(sizeof(MapNode));
  m7->x = 2, m7->y = 3;
  MapNode *m8 = malloc(sizeof(MapNode));
  m8->x = 2, m8->y = 1;
  graph_add(g,m7);
  graph_add(g,m8);
  graph_connect(g, m3, m7, heuristic(m3, m7));
  graph_connect(g, m4, m8, heuristic(m4, m8));


  graph_connect(g, m6, m7, heuristic(m6, m7));
  graph_connect(g, m1, m7, heuristic(m1, m7));
  graph_connect(g, m5, m7, heuristic(m5, m7));

  graph_connect(g, m5, goal, heuristic(m5, goal));
  graph_connect(g, m7, goal, heuristic(m7, goal));
  graph_connect(g, m8, goal, heuristic(m8, goal));

  clock_t t_start_pathearch, t_end_pathearch;
  
  t_start_pathearch = clock();
  AStarPathNode *a = astar(g,start,goal,heuristic,hash);
  AStarPathNode *path = a;
  t_end_pathearch = clock();
  
  double total_time = ((double) (t_end_pathearch - t_start_pathearch)) / CLOCKS_PER_SEC;
  printf("Time taken in interval pathearch: %gs\n", total_time);
  
  do {
    print_node(a);
    printf("\n");
    a = a->parent;
  } while (a != NULL);

  astar_free_path(path);
  graph_destroy(g,free);

}

void example2() {

  

}

int main(int argc, const char** argv) {

  example1();

  return 0;
}
