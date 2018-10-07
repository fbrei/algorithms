#include "include/vgraph.h"
#include <stdio.h>

int compare_to(void* a,void* b) {
  return 1;
}

unsigned int equals(void *a, void *b) {
  return (a == b) ? 1 : 0;
}

Graph* vgraph_circular_obstacles(MapPoint *start, MapPoint *goal, DList *obstacles, double (*distance_metric)(void*, void*)) {

  const unsigned int DYNAMIC = 1;

  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g, start);
  graph_add(g, goal);


  if(DYNAMIC) {

    CircularObstacle *co = tangent_get_first_blocking(start, goal, obstacles, distance_metric);
    if(co == NULL) {
      graph_connect(g,start,goal,distance_metric(start,goal));
    } else {
      DList *out = tangent_circle_point_intersects(start, co);
      MapPoint *tmp_point = NULL;
      for(size_t idx = 0; idx < out->num_items; idx++) {
        tmp_point = darray_get(out->data, idx);
        if(!tangent_is_blocked(start, tmp_point, obstacles)) {
          tmp_point->is_in = 1;
          graph_add(g, tmp_point);
          graph_connect(g, start, tmp_point, distance_metric(start, tmp_point));
          _obstacle_add_map_point(co, tmp_point);
        }
      }

      PrQueue *open_obstacles = prqueue_init(compare_to);
      open_obstacles->equals = equals;

      prqueue_add(open_obstacles, co);

      CircularObstacle *current = NULL;
      while((current = prqueue_pop(open_obstacles)) != NULL) {
        // Try to reach goal directly
        DList *goal_to_current = tangent_circle_point_intersects(goal,current);

        // Now see if any of these connections is blocked. If so, add the corresponding
        // obstacle to the list
        MapPoint *next = NULL;
        while((next = dlist_iterate(goal_to_current,next)) != NULL) {
          graph_add(g,next);
          CircularObstacle *blocking = tangent_get_first_blocking(next,goal,obstacles,distance_metric);
          if(blocking == NULL) {
            graph_connect(g,next,goal,distance_metric(next,goal));
            _obstacle_add_map_point(current,next);
          } else if(!prqueue_contains(open_obstacles, blocking)){
            prqueue_add(open_obstacles, blocking);
            DList *intersections = tangent_circle_intersects(current,blocking);
            MapPoint *source = NULL, *target = NULL;
            while(1) {
              source = dlist_iterate(intersections, target);
              if(source == NULL) { break; }
              target = dlist_iterate(intersections, source);

              if(!tangent_is_blocked(source,target,obstacles)) {
                source->is_in = 0;
                target->is_in = 1;
                _obstacle_add_map_point(current,source);
                _obstacle_add_map_point(blocking,target);

                graph_add(g,source);
                graph_add(g,target);
                graph_connect(g,source,target,distance_metric(source,target));
              }
            }
          }
        }
      }

    }
  } else {

    CircularObstacle *tmp_obstacle = NULL;
    DList *out = NULL;

    if(!tangent_is_blocked(start,goal,obstacles)) {
      graph_connect(g,start,goal,distance_metric(start,goal));
    }

    while((tmp_obstacle = (CircularObstacle*) dlist_iterate(obstacles, tmp_obstacle)) != NULL) {

      // ------------------------------------------------------------------------------

      out = tangent_circle_point_intersects(start, tmp_obstacle);
      MapPoint *tmp_point = NULL;
      while((tmp_point = (MapPoint*) dlist_iterate(out, tmp_point)) != NULL) {
        if(tangent_is_blocked(start, tmp_point, obstacles)) {
          continue;
        }
        tmp_point->is_in = 1;

        _obstacle_add_map_point(tmp_obstacle, tmp_point);
        graph_add(g, tmp_point);
        graph_connect(g, start, tmp_point, distance_metric(start, tmp_point));
      }

      // ------------------------------------------------------------------------------

      out = tangent_circle_point_intersects(goal, tmp_obstacle);
      tmp_point = NULL;
      while((tmp_point = (MapPoint*) dlist_iterate(out, tmp_point)) != NULL) {
        if(tangent_is_blocked(goal, tmp_point, obstacles)) {
          continue;
        }
        tmp_point->is_in = 0;
        _obstacle_add_map_point(tmp_obstacle, tmp_point);
        graph_add(g, tmp_point);
        graph_connect(g, tmp_point, goal, distance_metric(goal, tmp_point));
      }

      // ------------------------------------------------------------------------------


      CircularObstacle *other_obstacle = NULL;

      while((other_obstacle = (CircularObstacle*) dlist_iterate(obstacles, other_obstacle)) != NULL) {
        if(other_obstacle == tmp_obstacle) break;

        out = tangent_circle_inner_intersects(tmp_obstacle, other_obstacle);
        MapPoint *first = NULL, *second = NULL;

        while(1) {
          first = dlist_iterate(out, second);
          if(first == NULL) break;
          second = dlist_iterate(out, first);

          if(tangent_is_blocked(first,second,obstacles)) {
            continue;
          }

          graph_add(g, first);
          graph_add(g, second);
          if(distance_metric(first,goal) < distance_metric(second,goal)) {
            first->is_in = 1;
            second->is_in = 0;
            graph_connect(g, second, first, distance_metric(first, second));
          } else {
            first->is_in = 0;
            second->is_in = 1;
            graph_connect(g, first, second, distance_metric(first, second));
          }


          _obstacle_add_map_point(tmp_obstacle, first);
          _obstacle_add_map_point(other_obstacle, second);
        }


        out = tangent_circle_outer_intersects(tmp_obstacle, other_obstacle);
        first = NULL, second = NULL;

        while(1) {
          first = dlist_iterate(out, second);
          if(first == NULL) break;
          second = dlist_iterate(out, first);

          if(tangent_is_blocked(first,second,obstacles)) {
            continue;

          }

          graph_add(g, first);
          graph_add(g, second);
          if(distance_metric(first,goal) < distance_metric(second,goal)) {
            first->is_in = 1;
            second->is_in = 0;
            graph_connect(g, second, first, distance_metric(first, second));
          } else {
            first->is_in = 0;
            second->is_in = 1;
            graph_connect(g, first, second, distance_metric(first, second));
          }

          _obstacle_add_map_point(tmp_obstacle, first);
          _obstacle_add_map_point(other_obstacle, second);
        }
      }

      // ------------------------------------------------------------------------------

    }
  }

  void *tmp = NULL;
  while((tmp = dlist_iterate(obstacles, tmp)) != NULL) {
    /* _obstacle_connect_map_points((CircularObstacle*) tmp, g, goal, distance_metric, obstacles); */
    /* _obstacle_connect_map_points((CircularObstacle*) tmp, g, NULL, NULL, obstacles); */
    /* _obstacle_connect_with_intermediate((CircularObstacle*) tmp, g, goal, distance_metric); */
    _obstacle_connect_directed_intermediates((CircularObstacle*) tmp, g, goal, distance_metric);
  }


  return g;
}
