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
} MapPoint;


/*! \struct _CircularObstacle 
 *  \brief Brief struct description
 *
 *  Detailed description
 */
typedef struct _CircularObstacle {
  MapPoint position;
  double radius;
} CircularObstacle;


/**
 * \brief Calculates the points where the tangent would touch the circle
 *
 * Use this method if all you need are the points where the tangent would
 * touch the circle (for exampling if you want to calculate a visibility
 * graph)
 *
 * \param point_x x-coordinate of the point
 * \param point_y y-coordinate of the point
 * \param circ_x x-coordinate of the circle
 * \param circ_y y-coordinate of the circle
 * \param circ_r Radius of the circle
 * \return A DArray containing the points of intersection
 */
DArray* tangent_circle_point_intersects(double point_x, double point_y,
    double circ_x, double circ_y, double circ_r);


/**
 * \brief Calculates the intersection points of the outer
 * tangents between two circles.
 *
 * Given two descriptions of circles, this function returns the points
 * where the outer tangent originating from the source touches the
 * target circle.
 * 
 * \param source_x x-coordinate of source circle
 * \param source_y x-coordinate of source circle
 * \param source_r Radius of source circle
 * \param target_x x-coordinate of target circle
 * \param target_y x-coordinate of target circle
 * \param target_r Radius of target circle
 * \return A DArray containing the points of origin and intersection in pairs
 */
DArray* tangent_circle_outer_intersects(double source_x, double source_y, double source_r,
    double target_x, double target_y, double target_r);

/**
 * \brief Calculates the intersection points of the outer
 * tangents between two circles.
 *
 * Given two descriptions of circles, this function returns the points
 * where the outer tangent originating from the source touches the
 * target circle.
 * 
 * \param source_x x-coordinate of source circle
 * \param source_y x-coordinate of source circle
 * \param source_r Radius of source circle
 * \param target_x x-coordinate of target circle
 * \param target_y x-coordinate of target circle
 * \param target_r Radius of target circle
 * \return A DArray containing the points of origin and intersection in pairs
 */
DArray* tangent_circle_inner_intersects(double source_x, double source_y, double source_r,
    double target_x, double target_y, double target_r);

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

