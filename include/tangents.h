/**
 *	\file tangents.h
 *  \brief Calculation of tangents on circles
 *
 *	Collection of functions to calculate the tangents on a circle
 *	given either another circle or a point outside of the circle
 *
 *	\date 2018/09/05
 *	\author Felix Brei
 *
 *	\bug In tangent_circle_point: need to handle case whre circ_y == point_y
 */

#ifndef TANGENTS_H_UJO8EG29
#define TANGENTS_H_UJO8EG29

#include "dtype.h"


/** \brief Describes a point in a 2D-plant
 *
 *  Used to hold and group coordinates.
 */
typedef struct _MapPoint {
  double x;   /**< The x-coordinate */
  double y;   /**< The y-coordinate */
  double score;
} MapPoint;


/*! \struct _CircularObstacle 
 *  \brief Brief struct description
 *
 *  Detailed description
 */
typedef struct _CircularObstacle {
  MapPoint position;
  double radius;

  size_t _num_map_points;
  DArray *_map_points;
} CircularObstacle;


CircularObstacle* obstacle_init(double x, double y, double r);
void obstacle_destroy(void *c);

/**
 * \brief Register a map point with a circular object.
 *
 * This is an internal function and should never have
 * to be called by an outsider.
 *
 * \param *c The obstacle that the point lives on
 * \param *p The map point.
 */
void _obstacle_add_map_point(CircularObstacle *c, MapPoint *p);

void _obstacle_connect_map_points(CircularObstacle *c, Graph *g);

/**
 * \brief Calculates the points where the tangent would touch the circle
 *
 * Use this method if all you need are the points where the tangent would
 * touch the circle (for exampling if you want to calculate a visibility
 * graph)
 *
 * \param *p The point on the 2D map
 * \param *c The round obstacle / circle
 * \return A DArray containing the points of intersection
 */
DArray* tangent_circle_point_intersects(MapPoint *p, CircularObstacle *c);


/**
 * \brief Calculates the intersection points of the outer
 * tangents between two circles.
 *
 * Given two descriptions of circles, this function returns the points
 * where the outer tangent originating from the source touches the
 * target circle.
 * 
 * \param *c1 The first circle
 * \param *c2 The second circle
 * \return A DArray containing the points of origin and intersection in pairs
 */
DArray* tangent_circle_outer_intersects(CircularObstacle *c1, CircularObstacle *c2);


/**
 * \brief Calculates the intersection points of the inner
 * tangents between two circles.
 *
 * Given two descriptions of circles, this function returns the points
 * where the outer tangent originating from the source touches the
 * target circle.
 * 
 * \param *c1 The first circle
 * \param *c2 The second circle
 * \return A DArray containing the points of origin and intersection in pairs
 */
DArray* tangent_circle_inner_intersects(CircularObstacle *c1, CircularObstacle *c2);


/**
 * \brief Calculates if the path between two points is blocked
 * by an obstacle
 *
 * Given two points, we need to know if the path between them is passable
 * for our route planning problem.
 * 
 * \param *p1 The starting point
 * \param *p2 The goal point
 * \param *obstacles The obstacles on the map
 * \return 1 if pass is block (cannot be traversed), 0 if it is free
 */
unsigned short tangent_is_blocked(MapPoint *p1, MapPoint *p2, DArray *obstacles);

#endif /* end of include guard: TANGENTS_H_UJO8EG29 */

