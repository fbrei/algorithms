#include "vgraph.h"
#include <stdio.h>

Graph* vgraph_circular_obstacles(MapPoint *start, MapPoint *goal, DList *obstacles, double (*distance_metric)(void*, void*)) {

  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g, start);
  graph_add(g, goal);


  CircularObstacle *tmp_obstacle = NULL;
  DList *out = NULL;
  while((tmp_obstacle = (CircularObstacle*) dlist_iterate(obstacles, tmp_obstacle)) != NULL) {

    // ------------------------------------------------------------------------------

    out = tangent_circle_point_intersects(start, tmp_obstacle);
    MapPoint *tmp_point = NULL;
    while((tmp_point = (MapPoint*) dlist_iterate(out, tmp_point)) != NULL) {
      if(tangent_is_blocked(start, tmp_point, obstacles)) continue;

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
        graph_connect(g, second, first, distance_metric(first, second));

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
        graph_connect(g, second, first, distance_metric(first, second));

        _obstacle_add_map_point(tmp_obstacle, first);
        _obstacle_add_map_point(other_obstacle, second);
      }
    }

    // ------------------------------------------------------------------------------

    void *tmp = NULL;
    while((tmp = dlist_iterate(obstacles, tmp)) != NULL) {
      _obstacle_connect_map_points((CircularObstacle*) tmp, g);
    }
  }

  return g;
}
