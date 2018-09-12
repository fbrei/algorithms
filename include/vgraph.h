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

#include "graph.h"
#include "tangents.h"

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
    double (*distance_metric)(void*, void*));


#endif /* end of include guard: VGRAPH_H_AWDEVYPA */
