

#ifndef ASTAR_H_RTVQ7UZ5
#define ASTAR_H_RTVQ7UZ5

#include "dtype.h"

/**
 * \brief Struct that holds A* data
 *
 * Contains a pointer to the actual data as well as some
 * meta information that is relevant for computing the best
 * path.
 *
 * This struct is not meant to be used directly by the user,
 * but it is being returned by the algorithm to represent
 * the solution. The definition here should allow the
 * user of the program to inspect what attributes the
 * object has.
 */
typedef struct _AStarPathNode {
  void *data; 		                /**< The actual node data */
  double total_dist; 	            /**< The distance traveled so far */
  double estimate; 		            /**< The expected distance to the goal */
  
  struct _AStarPathNode* parent;  /**< The parent node in the best (final) path */
} AStarPathNode;

/**
 * \brief A*
 *
 * A*
 * 
 * \param *g Graph
 * \param *start Start node
 * \param *goal Goal node
 * \return Path
 */
AStarPathNode* astar(Graph *g, void *start, void *goal, double (*heuristic)(void*,void*), int (*equals)(void*,void*) );

int _astar_compare_to(void *first, void *second);

#endif /* end of include guard: ASTAR_H_RTVQ7UZ5 */
