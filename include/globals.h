#ifndef GLOBALS_H_WZOSBK84
#define GLOBALS_H_WZOSBK84

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
  short is_in; /**< Incoming (1) or outgoing point (0) */
} MapPoint;



#endif /* end of include guard: GLOBALS_H_WZOSBK84 */
