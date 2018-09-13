#ifndef ASTAR_H_RTVQ7UZ5
#define ASTAR_H_RTVQ7UZ5

#include "../lib/dtypes/include/dtype.h"

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
  void *data;        /**< The actual node data */
  double total_dist; /**< The distance traveled so far */
  double estimate;   /**< The expected distance to the goal */

  struct _AStarPathNode
      *parent; /**< The parent node in the best (final) path */
} AStarPathNode;

/**
 * \brief A* implementation for a graph structure
 *
 * Runs A* on a graph. Only the memory blocks that this
 * funtion generates will also be freed. It returns
 * a pointer to the goal node that can be traversed backwards
 * by repeatedly selecting the parent attribute.
 *
 * \param *g Graph
 * \param *start Start node
 * \param *goal Goal node
 * \param (*heuristic)(void*,void*) The heuristic to use. Can be NULL but it
 * shouldn't \param (*hash)(void*) An optional hash functions. Can be NULL, but
 * supplying one increases speed. \return Pointer to the goal node that can be
 * traversed using the parent pointer.
 */
AStarPathNode *astar(Graph *g, void *start, void *goal,
                     double (*heuristic)(void *, void *),
                     unsigned long (*hash)(void *));

/**
 * \brief Frees an A* path.
 *
 * This method is convenience. It does not use any
 * special magic.
 *
 * \param *a The pointer to the path.
 */
void astar_free_path(AStarPathNode *a);

/**
 * \brief Internal compare-to-method
 *
 * Compares to A' nodes by looking at their scores
 * (total_dist + estimate)
 *
 * \param *first The first node
 * \param *second The second node
 * \return -1, 0, or 1 as usual
 */
int _astar_compare_to(void *first, void *second);

/**
 * \brief Checks i two A' nodes represent the same graph node
 *
 * Works by seeing if both their data pointer point to
 * the same address
 *
 * \param *first The first node
 * \param *second The second node
 * \return 1 if equal, 0 otherwise
 */
unsigned int _astar_equals(void *first, void *second);

/**
 * \brief The hash to use if none is provided.
 *
 * If you do not supply a hash function but NULL instead,
 * this hash function will be chosen. It always returns 0,
 * effectively turning the hash set into some sort of queue.
 *
 * \param *node The node to be hashed
 * \return Always 0
 */
unsigned long null_hash(void *node);

/**
 * \brief The empty heuristic: h(n) = 0 for all n
 *
 * If you do nt want to supply a heuristic funtion
 * you can use NULL to turn A* practically into Dijkstra
 *
 * \param *first The first node
 * \param *second The second node
 * \return Always 0
 */
double null_heuristic(void *first, void *second);

#endif /* end of include guard: ASTAR_H_RTVQ7UZ5 */
