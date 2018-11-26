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

#endif /* end of include guard: POLYGONS_H_MQ7DFOAE */
