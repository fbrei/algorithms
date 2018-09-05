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

/**
 * \brief Describes a linear function by its slope and y-intercept
 *
 * Since all tangents are linear functions it makes sense to create this
 * struct to group the results together. All functions in this module
 * will return a DArray containing exactly this datatype.
 *
 */
typedef struct _LinearFunction {
  double m; 		/**< Slope */
  double n; 		/**< y-intercept */
} LinearFunction;


/**
 * \brief Calculates the tangents on a circle given an external point
 *
 * Please note that this method will fail if the y-coordinate of the center
 * of the circle and the external point are the same, due to internal
 * calculations. This will be fixed in a later version and is currently
 * one of the top priorities.
 * 
 * \param point_x x-coordinate of the point
 * \param point_y y-coordinate of the point
 * \param circ_x x-coordinate of the circle
 * \param circ_y y-coordinate of the circle
 * \param circ_r Radius of the circle
 * \return A DArray containing the results
 */
DArray* tangent_circle_point(double point_x, double point_y, double circ_x, double circ_y, double circ_r);

#endif /* end of include guard: TANGENTS_H_UJO8EG29 */
