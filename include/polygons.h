/**
 *	\file polygons.h
 *  \brief Header file for polygonal obstacles
 *
 *	Use this file if you want to include polygonal obstacles in your
 *	visibility graph
 *
 *	\date 2018/11/21
 *	\author Felix Brei
 *
 *	\bug No known bugs.
 */

#ifndef POLYGONS_H_MQ7DFOAE
#define POLYGONS_H_MQ7DFOAE

#include "include/globals.h"
#include "lib/dtypes/include/dtype.h"

/*! \struct _PolygonalObstacle 
 *  \brief A polygonal obstacle in a 2D plane
 *
 *  A polygonal obstacle. It is defined by its corners
 *  (DList of MapPoints)
 */
typedef struct _PolygonalObstacle {
  DList *corners;
} PolygonalObstacle;


unsigned short polygon_is_blocked(MapPoint *p1, MapPoint *p2, DList *obstacles);

void polygon_destroy(PolygonalObstacle *po);

/**
 * \brief Brief description
 *
 * Long description
 * 
 * \param from 
 * \param to 
 * \param obstacles 
 * \param self 
 * \return Return value, delete if void
 */
PolygonalObstacle* polygon_get_first_blocking(MapPoint* from, MapPoint* to, DList* obstacles, PolygonalObstacle* self);

PolygonalObstacle* convex_hull(DList* map_points);

/**
 * \brief Calculates the area of a polygon.
 *
 * Note that this only works if the polygon contains no twists or
 * overlaps itself
 * 
 * \param *p The polygon of which the area should be calculated
 * \return The area of the given polygon
 */
double polygon_area(PolygonalObstacle *p);


/**
 * \brief Calculates the percentage of map that is covered
 *
 * See short descr.
 * 
 * \param *polygons The list of all polygons
 * \param width The width of the map
 * \param height The height of the map
 * \return Percentage of covered area (0..1)
 */
double polygon_map_covered(DList *polygons, double width, double height);

/**
 * \brief Checks if two polygons overlap (1 yes, 0 no)
 *
 * This is also 1 if one of the polygons lies completely inside the other one.
 * 
 * \param *p One of the polygons
 * \param *o The other polygons
 * \return 1 if p and o overlap, 0 otherwise
 */
unsigned short polygon_overlapping(PolygonalObstacle *p, PolygonalObstacle *o);

unsigned short _lines_intersect(MapPoint *from_a, MapPoint *to_a, MapPoint *from_b, MapPoint *to_b, double *r, double *s);

#endif /* end of include guard: POLYGONS_H_MQ7DFOAE */
