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

#include "lib/dtypes/include/dtype.h"
#include "include/globals.h"

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
  HSet *points;
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


/**
 * \brief Calculates the points where the tangent would touch the circle
 *
 * Use this method if all you need are the points where the tangent would
 * touch the circle (for exampling if you want to calculate a visibility
 * graph)
 *
 * \param *p The point on the 2D map
 * \param *c The round obstacle / circle
 * \return A DList containing the points of intersection
 */
DList* tangent_circle_point_intersects(MapPoint *p, CircularObstacle *c);


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
 * \return A DList containing the points of origin and intersection in pairs
 */
DList* tangent_circle_outer_intersects(CircularObstacle *c1, CircularObstacle *c2);


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
 * \return A DList containing the points of origin and intersection in pairs
 */
DList* tangent_circle_inner_intersects(CircularObstacle *c1, CircularObstacle *c2);


/**
 * \brief Calculates all intersection points of tangents between two circles
 *
 * Combination of both calls to inner and outer intersects, returns one combinded
 * list where pairs of map points consist of source and target
 * 
 * \param *c1 The originating obstace
 * \param *c2 The target obstacle
 * \return DList containing all start and end points of tangents
 */
DList* tangent_circle_intersects(CircularObstacle *c1, CircularObstacle *c2);

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
unsigned short tangent_is_blocked(MapPoint *p1, MapPoint *p2, DList *obstacles);

CircularObstacle* tangent_get_first_blocking(MapPoint *from, MapPoint *to, DList *obstacles, void *self, double (*distance)(void*,void*));
DList* tangent_get_blocking(MapPoint *from, MapPoint *to, DList *obstacles, void *self);
void _obstacle_connect_map_points(CircularObstacle *c, Graph *g, MapPoint *goal, double (*heuristic)(void*, void*), DList *other_obstacles);
void _obstacle_connect_with_intermediate(CircularObstacle *c, Graph *g, MapPoint *goal, double (*heuristic)(void*, void*));
void _obstacle_connect_directed_intermediates(CircularObstacle *c, Graph *g, MapPoint *goal, double (*heuristic)(void*, void*));
void _obstacle_sort_points(CircularObstacle *c, MapPoint *goal, double (*heuristic)(void*, void*));

#endif /* end of include guard: TANGENTS_H_UJO8EG29 */

