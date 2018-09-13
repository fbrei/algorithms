#include "../include/astar.h"

AStarPathNode* astar(Graph *g, void *start, void *goal, double (*heuristic)(void*,void*),
    unsigned long (*hash)(void*)) {

  if(hash==NULL) hash=null_hash;
  if(heuristic == NULL) heuristic = null_heuristic;

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
          continue;
        } else {
          AStarPathNode *old_neighbor = prqueue_get(frontier, tmp);
          if(old_neighbor == NULL) {
            prqueue_add(frontier, tmp);
          } else if(old_neighbor->total_dist + old_neighbor->estimate > tmp->total_dist + tmp->estimate) {
            prqueue_replace(frontier, old_neighbor, tmp);
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

  hset_destroy(explored, NULL);
  prqueue_destroy(frontier, NULL);
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

unsigned long null_hash(void *n) { return 0l; }
double null_heuristic(void *n, void *m) { return 0.0; }
