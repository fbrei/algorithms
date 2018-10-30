#include "include/vgraph.h"
#include "include/globals.h"
#include <stdio.h>

#define PRINTDEBUG 1

int compare_to(void* a,void* b) {
  UNUSED(a);
  UNUSED(b);
  return 1;
}

unsigned int equals(void *a, void *b) {
  return (a == b) ? 1 : 0;
}

void print_obstacle(FILE *stream, void *o) {
  CircularObstacle *c = (CircularObstacle*) o;
  fprintf(stream, "(%g,%g,%g)", c->position.x, c->position.y, c->radius);
}

void print_map_point(FILE *stream, void *o) {
  MapPoint *c = (MapPoint*) o;
  fprintf(stream, "(%g,%g)", c->x, c->y);
}

unsigned long obstacle_hash(void *v) {
  return (unsigned long) v;
}

Graph* vgraph_circular_obstacles(MapPoint *start, MapPoint *goal, DList *obstacles, double (*distance_metric)(void*, void*), const int DYNAMIC) {

  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g, start);
  graph_add(g, goal);


  if(DYNAMIC == 1) {

    CircularObstacle *co = tangent_get_first_blocking(start, goal, obstacles, NULL, distance_metric);
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
          CircularObstacle *blocking = tangent_get_first_blocking(next,goal,obstacles,current,distance_metric);
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
  } else if(DYNAMIC == 2) {

    CircularObstacle *co = tangent_get_first_blocking(start, goal, obstacles, NULL, distance_metric);
    PrQueue *local, *front;
    HSet *expanded, *local_expanded;

    local = prqueue_init(compare_to);
    prqueue_add(local, co);
    local->equals = equals;

    front = prqueue_init(compare_to);
    front->equals = equals;

    expanded = hset_init(NULLHASH,equals);

    // Initialization from the starting point
    if(co == NULL) {
      graph_connect(g,start,goal,distance_metric(start,goal));
    } else {

      prqueue_add(local,co);

      CircularObstacle *current = NULL;
      while((current = (CircularObstacle*) prqueue_pop(local)) != NULL) {
        prqueue_add(front, current);
        DList *out = tangent_circle_point_intersects(start, current);
        MapPoint *tmp_point = NULL;
        for(size_t idx = 0; idx < out->num_items; idx++) {
          tmp_point = darray_get(out->data, idx);
          DList *blocking = tangent_get_blocking(start, tmp_point, obstacles, NULL);
          if(blocking == NULL) {
            _obstacle_add_map_point(current,tmp_point);
            tmp_point->is_in = 1;
            graph_add(g,tmp_point);
            graph_connect(g,start,tmp_point,distance_metric(tmp_point, start));
          } else {
            free(tmp_point);
            CircularObstacle *tmp_obstacle = NULL;
            while((tmp_obstacle = dlist_iterate(blocking, tmp_obstacle))) {
              prqueue_add(local, tmp_obstacle);
            }
          }
        }
        dlist_destroy(out,NULL);
      }
    }

    // The front is initialized, now we keep expanding the front until nodes are dealt with

    CircularObstacle *current = NULL;

    while((current = (CircularObstacle*) prqueue_pop(front)) != NULL) {

      if(hset_contains(expanded,current) != -1) {
        continue;
      } else {
        hset_add(expanded,current);
      }

#if PRINTDEBUG == 1
      fprintf(stderr, "Expanding ");
      print_obstacle(stderr, current);
      fprintf(stderr, "\n");
#endif

      // Again local Initialization
      DList *out = tangent_circle_point_intersects(goal, current);
      for(size_t idx = 0; idx < out->num_items; idx++) {
        MapPoint *tmp_point = darray_get(out->data, idx);
        DList *blocking = tangent_get_blocking(tmp_point, goal, obstacles, current);
        if(blocking == NULL) {
          graph_add(g, tmp_point);
          tmp_point->is_in = 0;
          _obstacle_add_map_point(current,tmp_point);
          graph_connect(g,tmp_point,goal,distance_metric(goal,tmp_point));
        } else {
          free(tmp_point);
          CircularObstacle *tmp_obstacle = NULL;
          while((tmp_obstacle = dlist_iterate(blocking, tmp_obstacle))) {
            if(!prqueue_contains(local,tmp_obstacle)) {
              prqueue_add(local, tmp_obstacle);
            }
          }
        }
      }
      dlist_destroy(out,NULL);

      // Work through all blocking obstacles
      CircularObstacle *blocking = NULL;
      local_expanded = hset_init(obstacle_hash,equals);
      while((blocking = (CircularObstacle*) prqueue_pop(local)) != NULL) {

        if(hset_contains(local_expanded,blocking) != -1) {
          continue;
        }

        hset_add(local_expanded,blocking);

        if(!prqueue_contains(front, blocking)) {
          prqueue_add(front,blocking);
        }

#if PRINTDEBUG == 1
        fprintf(stderr, "... seeing ");
        print_obstacle(stderr,blocking);
        fprintf(stderr, "\n");
#endif

        out = tangent_circle_intersects(current, blocking);
        for(size_t ii = 0; ii < out->num_items; ii+=2) {
          MapPoint *from = darray_get(out->data,ii);
          MapPoint *to = darray_get(out->data,ii+1);
          CircularObstacle *next_blocking = tangent_get_first_blocking(from, to, obstacles, current, distance_metric);
          if(next_blocking == NULL) {

            _obstacle_add_map_point(current,from);
            _obstacle_add_map_point(blocking,to);
            from->is_in = 0;
            to->is_in = 1;
#if PRINTDEBUG == 1
            fprintf(stderr, "...... adding connection ");
            print_map_point(stderr, from);
            fprintf(stderr, " -> ");
            print_map_point(stderr, to);
            fprintf(stderr, "\n");
#endif

            graph_add(g,from);
            graph_add(g,to);
            graph_connect(g,from,to,(distance_metric(from,to)));
          } else {

#if PRINTDEBUG == 1
            fprintf(stderr, "...... blocked by ");
            print_obstacle(stderr,next_blocking);
            fprintf(stderr, "\n");
#endif

            if(!prqueue_contains(local, next_blocking)) {
              prqueue_add(local, next_blocking);
            }
            free(from);
            free(to);
          }
        }
        dlist_destroy(out,NULL);
      }
      hset_destroy(local_expanded,NULL);
      local_expanded = NULL;
    }

    prqueue_destroy(local, NULL);
    prqueue_destroy(front, NULL);
    hset_destroy(expanded,NULL);

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
          free(tmp_point);
          continue;
        }
        tmp_point->is_in = 1;

        _obstacle_add_map_point(tmp_obstacle, tmp_point);
        graph_add(g, tmp_point);
        graph_connect(g, start, tmp_point, distance_metric(start, tmp_point));
      }
      dlist_destroy(out,NULL);

      // ------------------------------------------------------------------------------

      out = tangent_circle_point_intersects(goal, tmp_obstacle);
      tmp_point = NULL;
      while((tmp_point = (MapPoint*) dlist_iterate(out, tmp_point)) != NULL) {
        if(tangent_is_blocked(goal, tmp_point, obstacles)) {
          free(tmp_point);
          continue;
        }
        tmp_point->is_in = 0;
        _obstacle_add_map_point(tmp_obstacle, tmp_point);
        graph_add(g, tmp_point);
        graph_connect(g, tmp_point, goal, distance_metric(goal, tmp_point));
      }
      dlist_destroy(out,NULL);

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
            free(first);
            free(second);
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
        dlist_destroy(out,NULL);


        out = tangent_circle_outer_intersects(tmp_obstacle, other_obstacle);
        first = NULL, second = NULL;

        while(1) {
          first = dlist_iterate(out, second);
          if(first == NULL) break;
          second = dlist_iterate(out, first);

          if(tangent_is_blocked(first,second,obstacles)) {
            free(first);
            free(second);
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

        dlist_destroy(out,NULL);
      }

      // ------------------------------------------------------------------------------

    }
  }

  void *tmp = NULL;
  while((tmp = dlist_iterate(obstacles, tmp)) != NULL) {
    _obstacle_connect_map_points((CircularObstacle*) tmp, g, goal, distance_metric, obstacles);
    /* _obstacle_connect_map_points((CircularObstacle*) tmp, g, NULL, NULL, obstacles); */
    /* _obstacle_connect_with_intermediate((CircularObstacle*) tmp, g, goal, distance_metric); */
    /* _obstacle_connect_directed_intermediates((CircularObstacle*) tmp, g, goal, distance_metric); */
  }


  return g;
}
