#include "astar.h"

#include <stdio.h>


AStarPathNode* astar(Graph *g, void *start, void *goal, double (*heuristic)(void*,void*), int (*equals)(void*,void*) ) {
  printf("Hello, this is the A* algorithm!\n");

  PrQueue *frontier = prqueue_init(_astar_compare_to);

  AStarPathNode *astar_start = malloc(sizeof(AStarPathNode));

  astar_start->data = start;
  astar_start->total_dist = 0.0;
  astar_start->estimate = heuristic(start,goal);
  astar_start->parent = NULL;

  prqueue_add(frontier, astar_start);

  void *current = NULL;

  while(1) {
    current = prqueue_pop(frontier);
    if(equals(  ((AStarPathNode*) current)->data, goal ) ) {
      prqueue_destroy(frontier, free);
      return current;
    }

    DArray *neighbors = graph_get_neighbors(g, ((AStarPathNode*) current)->data );
    if(neighbors != NULL) {

      void *new_neighbor = NULL;
      while((new_neighbor = darray_iterate(neighbors, new_neighbor)) != NULL) {
        AStarPathNode *tmp = malloc(sizeof(AStarPathNode));
        tmp->data = new_neighbor;
        tmp->total_dist = ((AStarPathNode*) current)->total_dist + *graph_get_edge_weight(g, ((AStarPathNode*) current)->data, new_neighbor);
        tmp->estimate = heuristic(tmp, goal);
        tmp->parent = current;

        prqueue_add(frontier, tmp);
      }
    }

    darray_destroy(neighbors, NULL);
  }

  return current;
}

int _astar_compare_to(void *first, void *second) {

  AStarPathNode *m1 = (AStarPathNode*) first;
  AStarPathNode *m2 = (AStarPathNode*) second;

  double score_diff = (m1->total_dist + m1->estimate) - (m2->total_dist + m2->estimate);

  if(score_diff > 0) {
    return 1;
  } else if(score_diff < 0) {
    return -1;
  } else {
    return 0;
  }

}
