#ifndef GLOBALS_H_WZOSBK84
#define GLOBALS_H_WZOSBK84

#include "include/dtype.h"

#define UNUSED(x) (void) x

/** \brief Describes a point in a 2D-plane
 *
 *  Used to hold and group coordinates.
 */
typedef struct _MapPoint {
  double x;   /**< The x-coordinate */
  double y;   /**< The y-coordinate */
  double score;
  double h;
  short is_in; /**< Incoming (1) or outgoing point (0). */
  DList *origins; /**< List of map points that can see this one. */
  PrQueue *local_queue; /**< List of map points that should be tried to be reached. */
  void *obstacle; /**< The obstacle that this point is sitting on (if any). Should be set to NULL otherwise. */
  DList *visited_points; /**< List of points that this map point has tried to reach so far. */
} MapPoint;



#endif /* end of include guard: GLOBALS_H_WZOSBK84 */
