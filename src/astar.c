#include "include/astar.h"

#include <stdio.h>

void print_node(void *n);

AStarPathNode* astar(Graph *g, void *start, void *goal, double (*heuristic)(void*,void*),
    unsigned long (*hash)(void*)) {


  if(hash==NULL) hash = NULLHASH;
  if(heuristic == NULL) heuristic = NULLHEURISTIC;

  PrQueue *frontier = prqueue_init(_astar_compare_to);
  frontier->equals = _astar_equals;
  HSet *explored = hset_init(hash, _astar_equals);

  AStarPathNode *astar_start = malloc(sizeof(AStarPathNode));

  astar_start->data = start;
  astar_start->total_dist = 0.0;
  astar_start->estimate = heuristic(start,goal);
  astar_start->parent = NULL;

  prqueue_add(frontier, astar_start);

  void *current = NULL;

  while(1) {
    current = prqueue_pop(frontier);
    if(current == NULL || ((AStarPathNode*) current)->data == goal ) {
      break;
    }
    if(hset_contains(explored,current) != -1) {
      free(current);
      continue;
    }
    hset_add(explored, current);

    DArray *neighbors = graph_get_neighbors(g, ((AStarPathNode*) current)->data );
    if(neighbors != NULL) {

      void *new_neighbor = NULL;
      while((new_neighbor = darray_iterate(neighbors, new_neighbor)) != NULL) {
        AStarPathNode *tmp = malloc(sizeof(AStarPathNode));

        tmp->data = new_neighbor;
        tmp->total_dist = ((AStarPathNode*) current)->total_dist + *graph_get_edge_weight(g, ((AStarPathNode*) current)->data, new_neighbor);
        tmp->estimate = heuristic(tmp, goal);
        tmp->parent = current;

        if(hset_contains(explored, tmp) != -1) {
          free(tmp);
        } else {
          if(prqueue_contains(frontier,tmp)) {
            AStarPathNode *current_best = prqueue_get(frontier,tmp);
            if(current_best->total_dist < tmp->total_dist) {
              free(tmp);
            } else {
              prqueue_replace(frontier,current_best,tmp);
            }
          } else {
            prqueue_add(frontier, tmp);
          }
        }
      }
      darray_destroy(neighbors, NULL);
    }

  }

  void *tmp = current;
  while(tmp != NULL) {
    hset_remove(explored, tmp);
    tmp = ((AStarPathNode*) tmp)->parent;
  }

  hset_destroy(explored, free);
  prqueue_destroy(frontier,free);
  return current;
}

void astar_free_path(AStarPathNode *a) {
  while(a != NULL) {
    AStarPathNode *tmp = a;
    a = a->parent;
    free(tmp);
  }
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

unsigned int _astar_equals(void *first, void* second) {

  AStarPathNode *m1 = (AStarPathNode*) first;
  AStarPathNode *m2 = (AStarPathNode*) second;

  return (m1->data == m2->data) ? 1 : 0;
}

double NULLHEURISTIC(void *first, void *second) {
  (void)(first), (void) second;
  return 0.0l;
}
