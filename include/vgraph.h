/**
 *	\file vgraph.h
 *  \brief Contains methods to calculate visibility graphs
 *
 *	Depending on the underlying structure, this file may provide
 *	just the right method to turn the map into a visibility
 *	graph that can be shoved right into A* for shortest path
 *	problems
 *
 *	\date 2018/09/12
 *	\author Felix Brei
 *
 *	\bug No known bugs.
 */

#ifndef VGRAPH_H_AWDEVYPA
#define VGRAPH_H_AWDEVYPA

#include "lib/dtypes/include/graph.h"
#include "include/tangents.h"
#include "include/polygons.h"

/*! \enum VGRAPH_MODES
 *
 *  Different dynamic or full modes of vgraph algo
 */
enum VGRAPH_MODES { 
  VGRAPH_FULL,
  VGRAPH_DYNAMIC,
  VGRAPH_DYNAMIC_SINGLE_ORIGIN,
  VGRAPH_DYNAMIC_LOCAL_PRUNING
};

/*! \enum OBSTACLE_TYPES
 *
 *  Identifies different types of obstacles
 */
enum OBSTACLE_TYPES { 
  SPHERICAL_OBSTACLE,
  POLYGONAL_OBSTACLE
};


/**
 * \brief Brief description
 *
 * Long description
 * 
 * \param *start 
 * \param *goal 
 * \param *obstacles 
 * \param  
 * \return Return value, delete if void
 */
Graph* vgraph_circular_obstacles(MapPoint *start, 
    MapPoint *goal, 
    DList *obstacles,
    double (*distance_metric)(void*, void*),
    const int dynamic);

Graph* vgraph_polygonal_obstacles(MapPoint *start, 
    MapPoint *goal, 
    DList *obstacles,
    double (*distance_metric)(void*, void*),
    const int dynamic);

Graph* vgraph(MapPoint *start, 
    MapPoint *goal, 
    DList *polygons,
    DList *spheres,
    double (*distance_metric)(void*, void*),
    const int dynamic,
    short verbose,
    int (*priority_func)(void*,void*)
    );

MapPoint *vstar(MapPoint *start,
    MapPoint *goal, 
    DList *polygons,
    DList *spheres,
    double (*distance_metric)(void*, void*),
    short verbose
    );

void* get_first_blocking(MapPoint *from, MapPoint *to, DList *spheres, DList *polygons, void *self, double (*distance_metric)(void*,void*), enum OBSTACLE_TYPES* type);
#endif /* end of include guard: VGRAPH_H_AWDEVYPA */
