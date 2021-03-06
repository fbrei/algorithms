#include "include/vgraph.h"
#include "include/globals.h"
#include <stdio.h>
#include <math.h>
/* *********************************
                 / )
                / /    _
      _        / /    / )
     ( `.     / /-.  / /
      `\ \   / // /`/ /
        ; `-`  (_/ / /
        |       (_/ /
        \          /
         )       /`
        /      /`
********************************** */

#ifndef PRINTDEBUG
#define PRINTDEBUG 0
#endif

#define PATH_MAX 9999999999.9l

/**
 * \brief Brief description
 *
 * Long description
 */
typedef struct _PrQueueObstacle {
  enum OBSTACLE_TYPES type; 		/**< description */
  void *data; 		/**< description */
} PrQueueObstacle;

int compare_to(void* a,void* b) {
  UNUSED(a);
  UNUSED(b);
  return 1;
}

int compare_to_2(void *a, void *b) {

  MapPoint *m = (MapPoint*) a;
  MapPoint *n = (MapPoint*) b;

  return (m->shortest_length < n->shortest_length) ? 0  : 1;
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

void print_polygon(FILE *stream, void *o) {
  PolygonalObstacle *p = (PolygonalObstacle*) o;
  for(size_t idx = 0; idx < p->corners->num_items; idx++) {
    MapPoint *c = darray_get(p->corners->data,idx);
    fprintf(stream, "(%g,%g) -> ", c->x, c->y);
  }
}

void add_all_corners(Graph *g, DList *polygons) {
  for(size_t ii = 0; ii < polygons->num_items; ii++) {
    PolygonalObstacle *p = darray_get(polygons->data,ii);
    for(size_t jj = 0; jj < p->corners->num_items; jj++) {
      MapPoint *m = darray_get(p->corners->data, jj);
      m->local_queue = NULL;
      m->origins = NULL;
      graph_add(g, m);
    }
  }
}

void _print_status_map(MapPoint *start, MapPoint *goal, DList *polygons) {

  fprintf(stderr, "(%+05g,%+05g)          ", start->x, start->y);
  for(size_t ii = 0; ii < polygons->num_items; ii++) {
    PolygonalObstacle *p = darray_get(polygons->data,ii);
    for(size_t jj = 0; jj < p->corners->num_items; jj++) {
      MapPoint *m = darray_get(p->corners->data, jj);
      fprintf(stderr, "(%+05g,%+05g)          ", m->x, m->y);
    }
  }
  fprintf(stderr, "(%+05g,%+05g)          ", goal->x, goal->y);
  fprintf(stderr, "\n");

  fprintf(stderr, "%g     (-)             ", start->shortest_length);
  for(size_t ii = 0; ii < polygons->num_items; ii++) {
    PolygonalObstacle *p = darray_get(polygons->data,ii);
    for(size_t jj = 0; jj < p->corners->num_items; jj++) {
      MapPoint *m = darray_get(p->corners->data, jj);
      if(m->shortest_ancestor) {
        fprintf(stderr, "%g     (%05g,%05g)               ", m->shortest_length, m->shortest_ancestor->x, m->shortest_ancestor->y);
      } else {
        fprintf(stderr, "%g     %p            ", m->shortest_length, (void*) m->shortest_ancestor);
      }
    }
  }
  if(goal->shortest_ancestor) {
    fprintf(stderr, "%g     (%05g,%05g)       ", goal->shortest_length, goal->shortest_ancestor->x, goal->shortest_ancestor->y);
  } else {
    
  }
  fprintf(stderr, "\n\n");

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

    DList *co = tangent_get_blocking(start, goal, obstacles, NULL);
    PrQueue *local, *front;
    HSet *expanded, *local_expanded;

    local = prqueue_init(compare_to);
    local->equals = equals;

    front = prqueue_init(compare_to);
    front->equals = equals;

    expanded = hset_init(NULLHASH,equals);

    // Initialization from the starting point
    if(co == NULL) {
      graph_connect(g,start,goal,distance_metric(start,goal));
    } else {

      for(size_t idx = 0; idx < co->num_items; idx++) {
#if PRINTDEBUG == 1
      fprintf(stderr, "Initializing with %p : ", darray_get(co->data,idx));
      print_obstacle(stderr, darray_get(co->data,idx));
      fprintf(stderr, "\n");
#endif
        prqueue_add(local, darray_get(co->data,idx));
      }

      CircularObstacle *current = NULL;
      local_expanded = hset_init(obstacle_hash,equals);
      while((current = (CircularObstacle*) prqueue_pop(local)) != NULL) {

        if(hset_contains(local_expanded, current) != -1) {
          continue;
        }

        hset_add(local_expanded, current);

        if(!prqueue_contains(front, current)) {
          prqueue_add(front, current);
        }

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
            for(size_t idx = 0; idx < blocking->num_items; idx++) {
              tmp_obstacle = darray_get(blocking->data,idx);
              if(!prqueue_contains(local, tmp_obstacle)) {
                prqueue_add(local, tmp_obstacle);
#if PRINTDEBUG == 1
                fprintf(stderr, "...... adding %p : ", darray_get(blocking->data,idx));
                print_obstacle(stderr, darray_get(blocking->data,idx));
                fprintf(stderr, "\n");
#endif
              }
            }
            dlist_destroy(blocking,NULL);
          }
        }
        dlist_destroy(out,NULL);
      }
      hset_destroy(local_expanded, NULL);
      dlist_destroy(co,NULL);
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
      fprintf(stderr, "Expanding %p : ", (void*) current);
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
          dlist_destroy(blocking,NULL);
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

Graph* vgraph_polygonal_obstacles(MapPoint *start, MapPoint *goal, DList *obstacles, double (*distance_metric)(void*, void*), const int dynamic) {
  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g, start);
  graph_add(g, goal);

  size_t n_obstacles = obstacles->num_items;

  if(dynamic == 2) {

    // We have to add all corners of all obstacles to the graph first
    // This is just O(n) so not all that bad, but we will fix this in
    // a later release nonetheless. For now it makes bookkeeping way
    // easier and allows to focus on the actual algorithm
    for(size_t ii = 0; ii < n_obstacles; ii++) {
      PolygonalObstacle *p = darray_get(obstacles->data, ii);
      for(size_t jj = 0; jj < p->corners->num_items; jj++) {
        graph_add(g, darray_get(p->corners->data,jj));
      }
    }

    // First try to reach the goal directly.
    // If this succeeds, we are already done,
    // We do not use the is_blocked method because
    // we do not want to recalculate which obstacle might
    // be blocking our path
    PolygonalObstacle *o = polygon_get_first_blocking(start,goal,obstacles,NULL);
    if(o == NULL) {
      graph_connect(g,start,goal,distance_metric(start,goal));
    } else {
      
      // There is no direct path, we need to dynamically iterate over the obstacles until
      // we have found one

      HSet *explored = hset_init(obstacle_hash, equals);
      HSet *local_explored = hset_init(obstacle_hash, equals);
      PrQueue *front = prqueue_init(compare_to);
      PrQueue *local = prqueue_init(compare_to);
      local->equals = equals;
      front->equals = equals;

      // First we expand our visibility cone from the start
      // We try to make a connection to the obstacle that is
      // blocking us and repeat the process if that conection
      // itself is again blocked.
      prqueue_add(local, o);
      while((o = prqueue_pop(local)) != NULL) {
      
        if(hset_contains(local_explored,o) != -1) {
          continue;
        } else {
          hset_add(local_explored,o);
        }

        if(!prqueue_contains(front,o)) {
          prqueue_add(front,o);
        }

        size_t n_corners = o->corners->num_items;
        for(size_t ii = 0; ii < n_corners; ii++) {
          MapPoint *corner = darray_get(o->corners->data,ii);
          if(!polygon_is_blocked(start,corner,obstacles)) {
            graph_connect(g,start,corner,distance_metric(start,corner));
          } else {
            PolygonalObstacle *p = polygon_get_first_blocking(start,corner,obstacles,o);
            if(!prqueue_contains(local,p)) {
              prqueue_add(local,p);
            }
          }
        }

      }

      // Now that we have initialized the list of interesting
      // obstacles, we repeat the above proces for each one of
      // them except that now we are dealing with full obstacles
      // instead of a single point.
      while((o = prqueue_pop(front)) != NULL) {

        if(hset_contains(explored,o) != -1) {
          continue;
        } else {
          hset_add(explored,o);
        }

        /* fprintf(stderr, "Expanding "); */
        /* print_polygon(stderr, o); */
        /* fprintf(stderr, "\n"); */

        // Initialize local queue by trying to reach the goal directly
        size_t n_corners = o->corners->num_items;
        for(size_t ii = 0; ii < n_corners; ii++) {
          MapPoint *corner = darray_get(o->corners->data,ii);


          if(!polygon_is_blocked(corner,goal,obstacles)) {
            graph_connect(g,corner,goal,distance_metric(corner,goal));
          } else {
            PolygonalObstacle *p = polygon_get_first_blocking(corner,goal,obstacles,o);
            if(!prqueue_contains(local,p) && p != NULL) {
              /* fprintf(stderr, "    Blocked by %p :", p); */
              /* print_polygon(stderr, p); */
              /* fprintf(stderr, "\n"); */
              prqueue_add(local,p);
            }
          }
        }

        // The Q has been prepared, now we can start
        // working on it
        PolygonalObstacle *p;
        local_explored = hset_init(obstacle_hash, equals);
        while((p = prqueue_pop(local)) != NULL) {
          if(hset_contains(local_explored,p) != -1) {
            continue;
          } else {
            hset_add(local_explored, p);
          }

          /* fprintf(stderr, "    Connecting to "); */
          /* print_polygon(stderr, p); */
          /* fprintf(stderr, "\n"); */

          if(!prqueue_contains(front,p)) {
            prqueue_add(front,p);
          }

          size_t n_p_corners = p->corners->num_items;
          for(size_t ii = 0; ii < n_corners; ii++) {
            for(size_t jj = 0; jj < n_p_corners; jj++) {
              MapPoint *from = darray_get(o->corners->data,ii);
              MapPoint *to = darray_get(p->corners->data,jj);

              if(!polygon_is_blocked(from,to,obstacles)) {
                graph_connect(g,from,to,distance_metric(from,to));
              } else {
                PolygonalObstacle *q = polygon_get_first_blocking(from,to,obstacles,NULL);
                if(q != o && q != p && !prqueue_contains(local,q)) {
                  prqueue_add(local,q);
                }
              }
            }
          }
        }

      }
    }

    

  } else {

    if(!polygon_is_blocked(start,goal,obstacles)) {
      graph_connect(g,start,goal,distance_metric(start,goal));
    }

    for(size_t ii = 0; ii < n_obstacles; ii++) {
      PolygonalObstacle *p = darray_get(obstacles->data,ii);
      size_t n_corners = p->corners->num_items;
      for(size_t idx = 0; idx < n_corners; idx++) {

        MapPoint *current = darray_get(p->corners->data,idx);
        graph_add(g,current);

        // First try to make a connection to the start
        if(!polygon_is_blocked(start, current, obstacles)) {
          graph_connect(g,start,current, distance_metric(start,current));
        }

        // Then see if the goal is visible
        
        if(!polygon_is_blocked(current, goal, obstacles)) {
          graph_connect(g,current,goal,distance_metric(goal,current));
        }
      }

    }

    // Now all corners are already added to the graph so we can try to
    // interconnect them

    for(size_t ii = 0; ii < n_obstacles; ii++) {
      PolygonalObstacle *p = darray_get(obstacles->data,ii);
      size_t n_p_corners = p->corners->num_items;
      for(size_t corner_ii = 0; corner_ii < n_p_corners; corner_ii++) {
        MapPoint *current = darray_get(p->corners->data,corner_ii);
        for(size_t jj = 0; jj < n_obstacles; jj++) {
          PolygonalObstacle *o = darray_get(obstacles->data,jj);
          if(o == p) continue;
          size_t n_o_corners = o->corners->num_items;
          for(size_t corner_jj = 0; corner_jj < n_o_corners; corner_jj++) {
            MapPoint *next = darray_get(o->corners->data,corner_jj);
            if(!polygon_is_blocked(current,next,obstacles)) {
              graph_connect(g,current,next,distance_metric(current,next));
            }
          }
        }
      }
    }

    // Finally we connect all corners of a polygon that are direct neighbors
    for(size_t ii = 0; ii < n_obstacles; ii++) {
      PolygonalObstacle *p = darray_get(obstacles->data,ii);
      size_t n_p_corners = p->corners->num_items;
      for(size_t jj = 0; jj < n_p_corners; jj++) {
        MapPoint *first = darray_get(p->corners->data,jj);
        MapPoint *second = darray_get(p->corners->data,(jj+1) % n_p_corners);
        if(distance_metric(first,goal) < distance_metric(second,goal)) {
          graph_connect(g,second,first,distance_metric(first,second));
        } else {
          graph_connect(g,first,second,distance_metric(first,second));
        }
      }
    }
    
  }

  return g;
}

void update_queues(MapPoint *current, MapPoint *target, PrQueue *global) {
    if(darray_find(current->visited_points->data, target) == -1) {
      prqueue_add(current->local_queue, target);
      if(!prqueue_contains(global, current)) {
        prqueue_add(global, current);
      }
    }
}

void update_queues_with_simple_queue(MapPoint *current, MapPoint *target, Queue *global) {
    if(darray_find(current->visited_points->data, target) == -1) {
      prqueue_add(current->local_queue, target);
      if(!queue_contains(global, current)) {
        queue_push(global, current);
      }
    }
}

void add_source_point(MapPoint *current, MapPoint *new_origin, PrQueue *global) {

  if(darray_find(current->origins->data, new_origin) == -1) {

    /* fprintf(stderr, "       Adding "); */
    /* print_map_point(stderr, new_origin); */
    /* fprintf(stderr, " as new origin to "); */
    /* print_map_point(stderr, current); */
    /* fprintf(stderr, " | %lu items added to new origin's inspection list:\n", current->visited_points->num_items); */

    dlist_push(current->origins, new_origin);
    for(size_t ii = 0; ii < current->visited_points->num_items; ii++) {
      MapPoint *next = darray_get(current->visited_points->data, ii);
      /* fprintf(stderr, "                  "); */
      /* print_map_point(stderr, next); */
      /* fprintf(stderr, "\n"); */
      update_queues(new_origin, next, global);
    }
  }

}

void add_source_point_with_simple_queue(MapPoint *current, MapPoint *new_origin, Queue *global) {

  if(darray_find(current->origins->data, new_origin) == -1) {

    dlist_push(current->origins, new_origin);
    for(size_t ii = 0; ii < current->visited_points->num_items; ii++) {
      MapPoint *next = darray_get(current->visited_points->data, ii);
      update_queues_with_simple_queue(new_origin, next, global);
    }
  }

}

void update_path_lengths(MapPoint *current, double new_length, double (*distance_metric) (void*,void*)) {

  current->shortest_length = new_length;

  DList *updated = dlist_init();

  for(size_t idx = 0; idx < current->reachable->num_items; idx++) {
    MapPoint *next = darray_get(current->reachable->data, idx);
    if(next->shortest_length < new_length + distance_metric(current,next)) {
      next->shortest_length = new_length + distance_metric(current,next);
      dlist_push(updated, next);
    }
  }

  for(size_t idx = 0; idx < updated->num_items; idx++) {
    MapPoint *next = darray_get(current->reachable->data, idx);
    update_path_lengths(next, new_length + distance_metric(current,next), distance_metric);
  }

  dlist_destroy(updated, NULL);
}

unsigned int equals_with_spheres(void *a, void *b) {

  PrQueueObstacle *m = (PrQueueObstacle*) a;
  PrQueueObstacle *n = (PrQueueObstacle*) b;

  return (m->data == n->data) ? 1  : 0;
}

unsigned long hash_with_spheres(void *a) {

  PrQueueObstacle *m = (PrQueueObstacle*) a;

  return obstacle_hash(m->data);
}

void init_map_point(MapPoint *m) {
  m->origins = dlist_init();
  m->local_queue = prqueue_init(compare_to);
  m->local_queue->equals = equals;
  m->reachable = dlist_init();
  m->visited_points = dlist_init();
  m->shortest_length = PATH_MAX;

}





Graph* vgraph_with_simple_queue(MapPoint *start, MapPoint *goal, DList *polygons, DList *spheres, double (*distance_metric)(void*, void*), const int dynamic, short VERBOSE, int (*priority_func)(void*,void*)) {

  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g,start);
  graph_add(g,goal);

  add_all_corners(g, polygons);

  for(size_t ii = 0; ii < g->node_list->num_items; ii++) {
    MapPoint *m = darray_get(g->node_list, ii);
    m->origins = dlist_init();
    m->local_queue = prqueue_init(compare_to);
    m->local_queue->equals = equals;
    m->reachable = dlist_init();
    m->visited_points = dlist_init();
    if(m != goal) {
      prqueue_add(m->local_queue, goal);
    }
    m->shortest_length = 9999999999.9l;
  }

  // =========================================================

  Queue *global = queue_init();
  queue_push(global,start);
  global->equals = equals;

  double shortest_to_goal = 9999999999.9l;

  MapPoint *from, *to;
  start->shortest_length = 0.0l;

  // Get the next item on the todo list
  while((from = queue_pop(global)) != NULL) {

    /* _print_status_map(start, goal, polygons); */

    if(VERBOSE >= 2) {
      fprintf(stderr, "Checking (%g,%g) -> %lu nodes to inspect | Obstacle %p | Shortest length: %g\n", from->x, from->y, from->local_queue->num_items, from->obstacle, from->shortest_length);
    }

    if(from == goal || from->local_queue->num_items == 0) continue;

    if(from->obstacle != NULL && from->local_queue->num_items  > 0) {
      PolygonalObstacle *p = from->obstacle;
      long tmp_idx = darray_find(p->corners->data, from);
      size_t idx = (tmp_idx > 0) ? tmp_idx : 0;
      MapPoint *n;
      double d;

      n = darray_get(p->corners->data, (idx+1) % p->corners->num_items);
      d = distance_metric(from,n);
      if(darray_find(from->reachable->data, n) == -1) {
        dlist_push(from->reachable, n);
      } 
      if(VERBOSE == 2) {
        fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
        fprintf(stderr, "             Shortest so far: %g\n", n->shortest_length);
        fprintf(stderr, "             Candidate: %g\n", from->shortest_length + d);
      }
      if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
        graph_connect(g, from, n, distance_metric(from,n));
        add_source_point_with_simple_queue(n, from, global);
        n->shortest_length = from->shortest_length + d;
        queue_push(global, n);
        for(size_t idx = 0; idx < n->reachable->num_items; idx++) {
          MapPoint *m = darray_get(n->reachable->data, idx);
          if(!prqueue_contains(n->local_queue, m)) {
            prqueue_add(n->local_queue, m);
          }
        }
      }

      n = darray_get(p->corners->data, (idx >0) ? idx - 1 : p->corners->num_items - 1);
      d = distance_metric(from,n);

      if(darray_find(from->reachable->data, n) == -1) {
        dlist_push(from->reachable, n);
      } 
      if(VERBOSE == 2) {
        fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
        fprintf(stderr, "             Shortest so far: %g\n", n->shortest_length);
        fprintf(stderr, "             Candidate: %g\n", from->shortest_length + d);
      }
      if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
        graph_connect(g, from, n, distance_metric(from,n));
        add_source_point_with_simple_queue(n, from, global);
        n->shortest_length = from->shortest_length + d;
        queue_push(global, n);
        for(size_t idx = 0; idx < n->reachable->num_items; idx++) {
          MapPoint *m = darray_get(n->reachable->data, idx);
          if(!prqueue_contains(n->local_queue, m)) {
            prqueue_add(n->local_queue, m);
          }
        }
      }
    }

    // The point may have some points listed that it should try to reach.
    while((to = prqueue_pop(from->local_queue)) != NULL) {

      if(from->obstacle && from->obstacle == to->obstacle) continue;

      if(VERBOSE >= 2) {
        fprintf(stderr, "    Trying to reach (%g,%g)\n", to->x, to->y);
      }

      // As always, try to make a direct connection first

      if(!polygon_is_blocked(from, to, polygons)) {
        double d = distance_metric(from,to);
        if(VERBOSE >= 2) {
          fprintf(stderr, "        Connection is possible!\n");
          fprintf(stderr, "           So far: %g\n", to->shortest_length);
          fprintf(stderr, "           Candidate: %g\n", from->shortest_length + d);
        }
        // Update forward
        if(!graph_get_edge_weight(g, from, to))  dlist_push(from->reachable, to);

        if(from->shortest_length + d < to->shortest_length && from->shortest_length + d < shortest_to_goal) {
          to->shortest_length = from->shortest_length + d;
          if(!queue_contains(global, to)) queue_push(global, to);
          if(to == goal) {
            shortest_to_goal = from->shortest_length + d;
            if(VERBOSE == 2) {
              fprintf(stderr, "               Found a way to the goal [%g | %g]\n", from->shortest_length + d, to->shortest_length);
            }
          }

          for(size_t idx = 0; idx < to->reachable->num_items; idx++) {
            MapPoint *m = darray_get(to->reachable->data, idx);
            if(!prqueue_contains(to->local_queue, m)) {
              prqueue_add(to->local_queue, m);
            }
          }


          if(!graph_get_edge_weight(g, from, to))  graph_connect(g, from, to, d);

          add_source_point_with_simple_queue(to, from, global);

          // Update backward
          if(VERBOSE >= 2) {
            fprintf(stderr, "      Need to update %lu origins ...\n", from->origins->num_items);
          }
          for(size_t ii = 0; ii < from->origins->num_items; ii++) {
            MapPoint *origin = darray_get(from->origins->data, ii);
            if(VERBOSE >= 2) {
              fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
            }
            update_queues_with_simple_queue(origin, to, global);
          }
         }
        

      } else {

        PolygonalObstacle *p = polygon_get_first_blocking(from, to, polygons, from->obstacle);

        if(VERBOSE >= 2) {
          if(p) {
            fprintf(stderr, "        Blocked by %p \n", (void*) p);
            double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
            double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
            fprintf(stderr, "              (%g,%g)\n", p_x, p_y);
          }
        }

        // Do the whole expansion process as above in the other method, starting from the
        // current point
        PrQueue *local = prqueue_init(compare_to);
        HSet *local_explored = hset_init(obstacle_hash, equals);
        local->equals = equals;
        prqueue_add(local, p);
        while((p = prqueue_pop(local)) != NULL) {

          if(hset_contains(local_explored, p) != -1) {
            continue;
          } else {
            hset_add(local_explored, p);
          }

          if(VERBOSE >= 2) {
            double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
            double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
            fprintf(stderr, "       Current obstacle: (%g,%g)\n", p_x, p_y);
          }

    
          if(p == from->obstacle) continue;

          for(size_t ii = 0; ii < p->corners->num_items; ii++) {

            MapPoint *corner = darray_get(p->corners->data, ii);

            if(VERBOSE >= 2) {
              fprintf(stderr, "      Trying to reach (%g,%g)\n", corner->x, corner->y);
            }

            if(!polygon_is_blocked(from, corner, polygons)) {

              // Update forward
              if(graph_get_edge_weight(g, from, corner)) continue;
              if(darray_find(from->reachable->data, corner) == -1) {
                dlist_push(from->reachable, corner);
              } 

              if(VERBOSE >= 2) {
                fprintf(stderr, "        Connecting to (%g,%g)!\n", corner->x, corner->y);
              }
    
              double d = distance_metric(from,corner);
              if(VERBOSE == 2) {
                fprintf(stderr, "             Shortest so far: %g | Candidate: %g\n", corner->shortest_length, from->shortest_length + d);
              }
              if(from->shortest_length + d < corner->shortest_length && from->shortest_length + d < shortest_to_goal) {
                /* update_path_lengths(corner, from->shortest_length + d, distance_metric); */
                corner->shortest_length = from->shortest_length + d;
                for(size_t idx = 0; idx < corner->reachable->num_items; idx++) {
                  MapPoint *m = darray_get(corner->reachable->data, idx);
                  if(!prqueue_contains(corner->local_queue, m)) {
                    prqueue_add(corner->local_queue, m);
                  }
                }
                graph_connect(g, from, corner, distance_metric(from,corner));

                queue_push(global, corner);
                add_source_point_with_simple_queue(corner, from, global);
                dlist_push(corner->origins, from);

                // Update backward
                for(size_t jj = 0; jj < from->origins->num_items; jj++) {
                  MapPoint *origin = darray_get(from->origins->data, jj);
                  if(VERBOSE >= 2) {
                    fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
                  }
                  update_queues_with_simple_queue(origin, corner, global);
                  queue_push(global, origin);
                }
              }
            } else {
              PolygonalObstacle *block = polygon_get_first_blocking(from, corner, polygons, from->obstacle);
              if(block != p && block != from->obstacle && !prqueue_contains(local, block)) {
                prqueue_add(local, block);
              }
            }
          }
        }
      }
    }
  }

  return g;
}





Graph* vgraph(MapPoint *start, MapPoint *goal, DList *polygons, DList *spheres, double (*distance_metric)(void*, void*), const int dynamic, short VERBOSE, int (*priority_func)(void*,void*)) {

  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g,start);
  graph_add(g,goal);

  // The list of spherical obstacles is not used yet but will
  // be incorporated later
  UNUSED(spheres);

  if(priority_func == NULL) {
    return vgraph_with_simple_queue(start, goal, polygons, spheres, distance_metric, dynamic, VERBOSE, priority_func);
  }

  if(dynamic == VGRAPH_DYNAMIC_LOCAL_PRUNING) {

    add_all_corners(g, polygons);

    for(size_t ii = 0; ii < g->node_list->num_items; ii++) {
      MapPoint *m = darray_get(g->node_list, ii);
      m->origins = dlist_init();
      m->local_queue = prqueue_init(compare_to);
      m->local_queue->equals = equals;
      m->reachable = dlist_init();
      m->visited_points = dlist_init();
      if(m != goal) {
        prqueue_add(m->local_queue, goal);
      }
      m->shortest_length = 9999999999.9l;
    }

    // =========================================================

    PrQueue *global = prqueue_init(priority_func);
    global->equals = equals;
    prqueue_add(global,start);

    double shortest_to_goal = 9999999999.9l;

    MapPoint *from, *to;
    start->shortest_length = 0.0l;

    // Get the next item on the todo list
    while((from = prqueue_pop(global)) != NULL) {

      /* _print_status_map(start, goal, polygons); */

      if(VERBOSE >= 2) {
        fprintf(stderr, "Checking (%g,%g) -> %lu nodes to inspect | Obstacle %p | Shortest length: %g\n", from->x, from->y, from->local_queue->num_items, from->obstacle, from->shortest_length);
      }

      if(from == goal || from->local_queue->num_items == 0) continue;

      if(from->obstacle != NULL && from->local_queue->num_items  > 0) {
        PolygonalObstacle *p = from->obstacle;
        long tmp_idx = darray_find(p->corners->data, from);
        size_t idx = (tmp_idx > 0) ? tmp_idx : 0;
        MapPoint *n;
        double d;

        n = darray_get(p->corners->data, (idx+1) % p->corners->num_items);
        d = distance_metric(from,n);
        if(darray_find(from->reachable->data, n) == -1) {
          dlist_push(from->reachable, n);
        } 
        if(VERBOSE == 2) {
          fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
          fprintf(stderr, "             Shortest so far: %g\n", n->shortest_length);
          fprintf(stderr, "             Candidate: %g\n", from->shortest_length + d);
        }
        if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
          graph_connect(g, from, n, distance_metric(from,n));
          add_source_point(n, from, global);
          n->shortest_length = from->shortest_length + d;
          prqueue_add(global, n);
          for(size_t idx = 0; idx < n->reachable->num_items; idx++) {
            MapPoint *m = darray_get(n->reachable->data, idx);
            if(!prqueue_contains(n->local_queue, m)) {
              prqueue_add(n->local_queue, m);
            }
          }
        }

        n = darray_get(p->corners->data, (idx >0) ? idx - 1 : p->corners->num_items - 1);
        d = distance_metric(from,n);

        if(darray_find(from->reachable->data, n) == -1) {
          dlist_push(from->reachable, n);
        } 
        if(VERBOSE == 2) {
          fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
          fprintf(stderr, "             Shortest so far: %g\n", n->shortest_length);
          fprintf(stderr, "             Candidate: %g\n", from->shortest_length + d);
        }
        if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
          graph_connect(g, from, n, distance_metric(from,n));
          add_source_point(n, from, global);
          n->shortest_length = from->shortest_length + d;
          prqueue_add(global, n);
          for(size_t idx = 0; idx < n->reachable->num_items; idx++) {
            MapPoint *m = darray_get(n->reachable->data, idx);
            if(!prqueue_contains(n->local_queue, m)) {
              prqueue_add(n->local_queue, m);
            }
          }
        }
      }

      // The point may have some points listed that it should try to reach.
      while((to = prqueue_pop(from->local_queue)) != NULL) {

        if(from->obstacle && from->obstacle == to->obstacle) continue;

        if(VERBOSE >= 2) {
          fprintf(stderr, "    Trying to reach (%g,%g)\n", to->x, to->y);
        }

        // As always, try to make a direct connection first

        if(!polygon_is_blocked(from, to, polygons)) {
          double d = distance_metric(from,to);
          if(VERBOSE >= 2) {
            fprintf(stderr, "        Connection is possible!\n");
            fprintf(stderr, "           So far: %g\n", to->shortest_length);
            fprintf(stderr, "           Candidate: %g\n", from->shortest_length + d);
          }
          // Update forward
          if(!graph_get_edge_weight(g, from, to))  dlist_push(from->reachable, to);

          if(from->shortest_length + d < to->shortest_length && from->shortest_length + d < shortest_to_goal) {
            to->shortest_length = from->shortest_length + d;
            if(!prqueue_contains(global, to)) prqueue_add(global, to);
            if(to == goal) {
              shortest_to_goal = from->shortest_length + d;
              if(VERBOSE == 2) {
                fprintf(stderr, "               Found a way to the goal [%g | %g]\n", from->shortest_length + d, to->shortest_length);
              }
            }

            for(size_t idx = 0; idx < to->reachable->num_items; idx++) {
              MapPoint *m = darray_get(to->reachable->data, idx);
              if(!prqueue_contains(to->local_queue, m)) {
                prqueue_add(to->local_queue, m);
              }
            }


            if(!graph_get_edge_weight(g, from, to))  graph_connect(g, from, to, d);

            add_source_point(to, from, global);

            // Update backward
            if(VERBOSE >= 2) {
              fprintf(stderr, "      Need to update %lu origins ...\n", from->origins->num_items);
            }
            for(size_t ii = 0; ii < from->origins->num_items; ii++) {
              MapPoint *origin = darray_get(from->origins->data, ii);
              if(VERBOSE >= 2) {
                fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
              }
              update_queues(origin, to, global);
            }
           }
          

        } else {

          PolygonalObstacle *p = polygon_get_first_blocking(from, to, polygons, from->obstacle);

          if(VERBOSE >= 2) {
            if(p) {
              fprintf(stderr, "        Blocked by %p \n", (void*) p);
              double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
              double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
              fprintf(stderr, "              (%g,%g)\n", p_x, p_y);
            }
          }

          // Do the whole expansion process as above in the other method, starting from the
          // current point
          PrQueue *local = prqueue_init(compare_to);
          HSet *local_explored = hset_init(obstacle_hash, equals);
          local->equals = equals;
          prqueue_add(local, p);
          while((p = prqueue_pop(local)) != NULL) {

            if(hset_contains(local_explored, p) != -1) {
              continue;
            } else {
              hset_add(local_explored, p);
            }

            if(VERBOSE >= 2) {
              double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
              double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
              fprintf(stderr, "       Current obstacle: (%g,%g)\n", p_x, p_y);
            }

      
            if(p == from->obstacle) continue;

            for(size_t ii = 0; ii < p->corners->num_items; ii++) {

              MapPoint *corner = darray_get(p->corners->data, ii);

              if(VERBOSE >= 2) {
                fprintf(stderr, "      Trying to reach (%g,%g)\n", corner->x, corner->y);
              }

              if(!polygon_is_blocked(from, corner, polygons)) {

                // Update forward
                if(graph_get_edge_weight(g, from, corner)) continue;
                if(darray_find(from->reachable->data, corner) == -1) {
                  dlist_push(from->reachable, corner);
                } 

                if(VERBOSE >= 2) {
                  fprintf(stderr, "        Connecting to (%g,%g)!\n", corner->x, corner->y);
                }
      
                double d = distance_metric(from,corner);
                if(VERBOSE == 2) {
                  fprintf(stderr, "             Shortest so far: %g | Candidate: %g\n", corner->shortest_length, from->shortest_length + d);
                }
                if(from->shortest_length + d < corner->shortest_length && from->shortest_length + d < shortest_to_goal) {
                  /* update_path_lengths(corner, from->shortest_length + d, distance_metric); */
                  corner->shortest_length = from->shortest_length + d;
                  for(size_t idx = 0; idx < corner->reachable->num_items; idx++) {
                    MapPoint *m = darray_get(corner->reachable->data, idx);
                    if(!prqueue_contains(corner->local_queue, m)) {
                      prqueue_add(corner->local_queue, m);
                    }
                  }
                  graph_connect(g, from, corner, distance_metric(from,corner));

                  prqueue_add(global, corner);
                  add_source_point(corner, from, global);
                  dlist_push(corner->origins, from);

                  // Update backward
                  for(size_t jj = 0; jj < from->origins->num_items; jj++) {
                    MapPoint *origin = darray_get(from->origins->data, jj);
                    if(VERBOSE >= 2) {
                      fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
                    }
                    update_queues(origin, corner, global);
                    prqueue_add(global, origin);
                  }
                }
              } else {
                PolygonalObstacle *block = polygon_get_first_blocking(from, corner, polygons, from->obstacle);
                if(block != p && block != from->obstacle && !prqueue_contains(local, block)) {
                  prqueue_add(local, block);
                }
              }
            }
          }
        }
      }
    }

    return g;
    
  } else if(dynamic == VGRAPH_DYNAMIC) {

    add_all_corners(g, polygons);

    for(size_t ii = 0; ii < g->node_list->num_items; ii++) {
      MapPoint *m = darray_get(g->node_list, ii);
      m->origins = dlist_init();
      m->local_queue = prqueue_init(compare_to);
      m->local_queue->equals = equals;
      m->visited_points = dlist_init();
      if(m != goal) {
        prqueue_add(m->local_queue, goal);
      }
    }

    // =========================================================

    PrQueue *global = prqueue_init(compare_to);
    global->equals = equals;
    prqueue_add(global,start);

    MapPoint *from, *to;
    start->shortest_length = 0.0l;

    // Get the next item on the todo list
    while((from = prqueue_pop(global)) != NULL) {

      /* _print_status_map(start, goal, polygons); */

      if(VERBOSE >= 2) {
        fprintf(stderr, "Checking (%g,%g) -> %lu nodes to inspect | Obstacle %p | Shortest length: %g\n", from->x, from->y, from->local_queue->num_items, from->obstacle, from->shortest_length);
      }

      if(from == goal || from->local_queue->num_items == 0) continue;

      if(from->obstacle != NULL && from->local_queue->num_items  > 0) {
        PolygonalObstacle *p = from->obstacle;
        long tmp_idx = darray_find(p->corners->data, from);
        size_t idx = (tmp_idx > 0) ? tmp_idx : 0;
        MapPoint *n;

        n = darray_get(p->corners->data, (idx+1) % p->corners->num_items);
        if(VERBOSE == 2) {
          fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
        }
        if(graph_get_edge_weight(g, from, n) == NULL) {
          graph_connect(g, from, n, distance_metric(from,n));
          add_source_point(n, from, global);
          prqueue_add(global, n);
        }

        n = darray_get(p->corners->data, (idx >0) ? idx - 1 : p->corners->num_items - 1);

        if(VERBOSE == 2) {
          fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
        }
        if(graph_get_edge_weight(g, from, n) == NULL) {
          graph_connect(g, from, n, distance_metric(from,n));
          add_source_point(n, from, global);
          prqueue_add(global, n);
        }
      }

      // The point may have some points listed that it should try to reach.
      while((to = prqueue_pop(from->local_queue)) != NULL) {

        if(from->obstacle && from->obstacle == to->obstacle) continue;
        
        if(darray_find(from->visited_points->data, to) > -1) continue;
        dlist_push(from->visited_points, to);

        if(VERBOSE >= 2) {
          fprintf(stderr, "    Trying to reach (%g,%g)\n", to->x, to->y);
        }

        // As always, try to make a direct connection first

        if(!polygon_is_blocked(from, to, polygons)) {
          double d = distance_metric(from,to);
          if(VERBOSE >= 2) {
            fprintf(stderr, "        Connection is possible!\n");
          }
          // Update forward
          if(!prqueue_contains(global, to)) prqueue_add(global, to);
          if(!graph_get_edge_weight(g, from, to))  graph_connect(g, from, to, d);

          add_source_point(to, from, global);

          // Update backward
          if(VERBOSE >= 2) {
            fprintf(stderr, "      Need to update %lu origins ...\n", from->origins->num_items);
          }
          for(size_t ii = 0; ii < from->origins->num_items; ii++) {
            MapPoint *origin = darray_get(from->origins->data, ii);
            if(VERBOSE >= 2) {
              fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
            }
            update_queues(origin, to, global);
          }
          

        } else {

          PolygonalObstacle *p = polygon_get_first_blocking(from, to, polygons, from->obstacle);

          if(VERBOSE >= 2) {
            if(p) {
              fprintf(stderr, "        Blocked by %p \n", (void*) p);
              double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
              double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
              fprintf(stderr, "              (%g,%g)\n", p_x, p_y);
            }
          }

          // Do the whole expansion process as above in the other method, starting from the
          // current point
          PrQueue *local = prqueue_init(compare_to);
          HSet *local_explored = hset_init(obstacle_hash, equals);
          local->equals = equals;
          prqueue_add(local, p);
          while((p = prqueue_pop(local)) != NULL) {

            if(hset_contains(local_explored, p) != -1) {
              continue;
            } else {
              hset_add(local_explored, p);
            }

            if(VERBOSE >= 2) {
              double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
              double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
              fprintf(stderr, "       Current obstacle: (%g,%g)\n", p_x, p_y);
            }

      
            if(p == from->obstacle) continue;

            for(size_t ii = 0; ii < p->corners->num_items; ii++) {

              MapPoint *corner = darray_get(p->corners->data, ii);

              if(VERBOSE >= 2) {
                fprintf(stderr, "      Trying to reach (%g,%g)\n", corner->x, corner->y);
              }

              if(!polygon_is_blocked(from, corner, polygons)) {

                // Update forward
                if(graph_get_edge_weight(g, from, corner)) continue;

                if(VERBOSE >= 2) {
                  fprintf(stderr, "        Connecting to (%g,%g)!\n", corner->x, corner->y);
                }
      
                graph_connect(g, from, corner, distance_metric(from,corner));

                prqueue_add(global, corner);
                add_source_point(corner, from, global);
                dlist_push(corner->origins, from);

                // Update backward
                for(size_t jj = 0; jj < from->origins->num_items; jj++) {
                  MapPoint *origin = darray_get(from->origins->data, jj);
                  if(VERBOSE >= 2) {
                    fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
                  }
                  update_queues(origin, corner, global);
                  prqueue_add(global, origin);
                }
              } else {
                PolygonalObstacle *block = polygon_get_first_blocking(from, corner, polygons, from->obstacle);
                if(block != p && block != from->obstacle && !prqueue_contains(local, block)) {
                  prqueue_add(local, block);
                }
              }
            }
          }
        }
      }
    }

    return g;
    
  } else if(dynamic == VGRAPH_WITH_SPHERES) {

    add_all_corners(g, polygons);

    for(size_t ii = 0; ii < g->node_list->num_items; ii++) {
      MapPoint *m = darray_get(g->node_list, ii);
      init_map_point(m);
      if(m != goal) {
        prqueue_add(m->local_queue, goal);
      }
    }

    // =========================================================

    PrQueue *global = prqueue_init(priority_func);
    global->equals = equals;
    prqueue_add(global,start);

    double shortest_to_goal = 9999999999.9l;

    MapPoint *from, *to;
    start->shortest_length = 0.0l;

    // Get the next item on the todo list
    while((from = prqueue_pop(global)) != NULL) {

      if(VERBOSE >= 2) {
        fprintf(stderr, "Checking (%g,%g) -> %lu nodes to inspect | Obstacle %p | Shortest length: %g\n", from->x, from->y, from->local_queue->num_items, from->obstacle, from->shortest_length);
      }

      if(from == goal || from->local_queue->num_items == 0) continue;

      // This block updates the connections to the neighbors of the current map point
      // in case it sits on a polygon because by definition points that are connected
      // by an edge of a polygon can reach each other (although mathematically speaking
      // they are actually blocked but this would cause a lot of headache otherwise)
      if(from->obstacle != NULL && from->on_circle == 0) {
        PolygonalObstacle *p = from->obstacle;
        long tmp_idx = darray_find(p->corners->data, from);
        size_t idx = (tmp_idx > 0) ? tmp_idx : 0;
        MapPoint *n;
        double d;

        n = darray_get(p->corners->data, (idx+1) % p->corners->num_items);
        d = distance_metric(from,n);
        if(darray_find(from->reachable->data, n) == -1) {
          dlist_push(from->reachable, n);
        } 
        if(VERBOSE == 2) {
          fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
          fprintf(stderr, "             Shortest so far: %g\n", n->shortest_length);
          fprintf(stderr, "             Candidate: %g\n", from->shortest_length + d);
        }
        if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
          graph_connect(g, from, n, distance_metric(from,n));
          add_source_point(n, from, global);
          n->shortest_length = from->shortest_length + d;
          prqueue_add(global, n);
          for(size_t idx = 0; idx < n->reachable->num_items; idx++) {
            MapPoint *m = darray_get(n->reachable->data, idx);
            if(!prqueue_contains(n->local_queue, m)) {
              prqueue_add(n->local_queue, m);
            }
          }
        }

        n = darray_get(p->corners->data, (idx >0) ? idx - 1 : p->corners->num_items - 1);
        d = distance_metric(from,n);

        if(darray_find(from->reachable->data, n) == -1) {
          dlist_push(from->reachable, n);
        } 
        if(VERBOSE == 2) {
          fprintf(stderr,"        Connecting edges (%g,%g) -> (%g,%g)\n", from->x, from->y, n->x, n->y);
          fprintf(stderr, "             Shortest so far: %g\n", n->shortest_length);
          fprintf(stderr, "             Candidate: %g\n", from->shortest_length + d);
        }
        if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
          graph_connect(g, from, n, distance_metric(from,n));
          add_source_point(n, from, global);
          n->shortest_length = from->shortest_length + d;
          prqueue_add(global, n);
          for(size_t idx = 0; idx < n->reachable->num_items; idx++) {
            MapPoint *m = darray_get(n->reachable->data, idx);
            if(!prqueue_contains(n->local_queue, m)) {
              prqueue_add(n->local_queue, m);
            }
          }
        }
      }
      // End of polygon edge neighbor update stuff
      // =============================================================================
      // Let the real magic happen below this point, this here is just bookkeeping
      //
      // =============================================================================


      // Global variable for reuse
      enum OBSTACLE_TYPES o_type;

      // The point may have some points listed that it should try to reach.
      while((to = prqueue_pop(from->local_queue)) != NULL) {

        // If they are sitting on the same obstacle, we will skip this calculation because this was handled above
        if(from->obstacle && from->obstacle == to->obstacle) continue;

        if(VERBOSE >= 2) {
          fprintf(stderr, "    Trying to reach (%g,%g)\n", to->x, to->y);
        }

        // As always, try to make a direct connection first

        if(!is_blocked(from, to, spheres, polygons)) {
          double d = distance_metric(from,to);
          if(VERBOSE >= 2) {
            fprintf(stderr, "        Connection is possible!\n");
            fprintf(stderr, "           So far: %g\n", to->shortest_length);
            fprintf(stderr, "           Candidate: %g\n", from->shortest_length + d);
          }
          // Update forward
          if(!graph_get_edge_weight(g, from, to))  dlist_push(from->reachable, to);

          if(from->shortest_length + d < to->shortest_length && from->shortest_length + d < shortest_to_goal) {
            to->shortest_length = from->shortest_length + d;
            if(!prqueue_contains(global, to)) prqueue_add(global, to);
            if(to == goal) {
              shortest_to_goal = from->shortest_length + d;
              if(VERBOSE == 2) {
                fprintf(stderr, "               Found a way to the goal [%g | %g]\n", from->shortest_length + d, to->shortest_length);
              }
            }

            for(size_t idx = 0; idx < to->reachable->num_items; idx++) {
              MapPoint *m = darray_get(to->reachable->data, idx);
              if(!prqueue_contains(to->local_queue, m)) {
                prqueue_add(to->local_queue, m);
              }
            }


            if(!graph_get_edge_weight(g, from, to))  graph_connect(g, from, to, d);

            add_source_point(to, from, global);

            // Update backward
            if(VERBOSE >= 2) {
              fprintf(stderr, "      Need to update %lu origins ...\n", from->origins->num_items);
            }
            for(size_t ii = 0; ii < from->origins->num_items; ii++) {
              MapPoint *origin = darray_get(from->origins->data, ii);
              if(VERBOSE >= 2) {
                fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
              }
              update_queues(origin, to, global);
            }
           }
          

        } else {

          void *fb = get_first_blocking(from, to, spheres, polygons, from->obstacle, distance_metric, &o_type);

          if(VERBOSE >= 2) {
            if(fb) {
              fprintf(stderr, "        Blocked by %p \n", (void*) fb);
            }
          }

          if(fb == from->obstacle || !fb) continue;

          // Do the whole expansion process as above in the other method, starting from the
          // current point
          PrQueue *local = prqueue_init(compare_to);
          HSet *local_explored = hset_init(hash_with_spheres, equals_with_spheres);
          local->equals = equals_with_spheres;

          PrQueueObstacle *po = malloc(sizeof(PrQueueObstacle));
          po->data = fb;
          po->type = o_type;
          prqueue_add(local, po);

          while((po = prqueue_pop(local)) != NULL) {

            fprintf(stderr, "   -> Checking %p->%p\n", (void*) po, (void*) po->data);
            if(hset_contains(local_explored, po->data) != -1) {
              continue;
            } else {
              hset_add(local_explored, po->data);
            }

            if(po->data == from->obstacle) continue;

            DList *targets;

            if(po->type == POLYGONAL_OBSTACLE) {

              targets = ((PolygonalObstacle*) po->data)->corners;

            } else {
              
              targets = tangent_circle_point_intersects(from, ((CircularObstacle*) po->data));
              for(size_t idx = 0; idx < targets->num_items; idx++) {
                MapPoint *m = (MapPoint*) darray_get(targets->data, idx);
                init_map_point(m);
                m->obstacle = fb;
                m->on_circle = 1;
                prqueue_add(m->local_queue, goal);
              }

            }

            fprintf(stderr, "     Targets initialized\n");
            
            PolygonalObstacle *p = (PolygonalObstacle*) po->data;
            for(size_t ii = 0; ii < targets->num_items; ii++) {

              MapPoint *corner = darray_get(targets->data, ii);

              if(VERBOSE >= 2) {
                fprintf(stderr, "      Trying to reach (%g,%g)\n", corner->x, corner->y);
              }

              if(!is_blocked(from, corner, spheres, polygons)) {

                if(po->type == POLYGONAL_OBSTACLE) {
                  // Update forward
                  if(graph_get_edge_weight(g, from, corner)) continue;
                  if(darray_find(from->reachable->data, corner) == -1) {
                    dlist_push(from->reachable, corner);
                  } 

                  if(VERBOSE >= 2) {
                    fprintf(stderr, "        Connecting to (%g,%g)!\n", corner->x, corner->y);
                  }
        
                  double d = distance_metric(from,corner);
                  if(VERBOSE == 2) {
                    fprintf(stderr, "             Shortest so far: %g | Candidate: %g\n", corner->shortest_length, from->shortest_length + d);
                  }
                  if(from->shortest_length + d < corner->shortest_length && from->shortest_length + d < shortest_to_goal) {
                    /* update_path_lengths(corner, from->shortest_length + d, distance_metric); */
                    corner->shortest_length = from->shortest_length + d;
                    for(size_t idx = 0; idx < corner->reachable->num_items; idx++) {
                      MapPoint *m = darray_get(corner->reachable->data, idx);
                      if(!prqueue_contains(corner->local_queue, m)) {
                        prqueue_add(corner->local_queue, m);
                      }
                    }
                    graph_connect(g, from, corner, distance_metric(from,corner));

                    prqueue_add(global, corner);
                    add_source_point(corner, from, global);
                    dlist_push(corner->origins, from);

                    // Update backward
                    for(size_t jj = 0; jj < from->origins->num_items; jj++) {
                      MapPoint *origin = darray_get(from->origins->data, jj);
                      if(VERBOSE >= 2) {
                        fprintf(stderr, "        Updating origin point: (%g,%g)\n", origin->x, origin->y);
                      }
                      update_queues(origin, corner, global);
                      prqueue_add(global, origin);
                    }
                  }
                } else if(po->type == SPHERICAL_OBSTACLE) {
                  CircularObstacle *co = (CircularObstacle*) po->data;
                  if(hset_contains(co->points, corner) == -1) {
                    graph_add(g, corner);
                    graph_connect(g, from, corner, distance_metric(from,corner));
                    hset_add(co->points, corner);
                    DList *new_points = tangent_circle_point_intersects(to, co);
                    for(size_t idx = 0; idx < new_points->num_items; idx++) {
                      MapPoint *m = darray_get(new_points->data, idx);
                      if(hset_contains(co->points, m) == -1) {
                        fprintf(stderr, "        Connecting to (%g,%g)!\n", m->x, m->y);
                        init_map_point(m);
                        m->obstacle = corner->obstacle;
                        m->shortest_length = from->shortest_length + distance_metric(from,m);
                        m->on_circle = 1;

                        graph_add(g, m);
                        graph_connect(g, corner, m, distance_metric(corner,m));

                        fprintf(stderr, "      Adding (%g,%g) to gloabl Q\n", m->x, m->y);
                        prqueue_add(global, m);
                        prqueue_add(m->local_queue, goal);
                        hset_add(co->points, m);
                      }
                    }
                  } else {
                    fprintf(stderr, "      Already checked\n");
                  }
                }
              } else {
                void *new_blocking = get_first_blocking(from, corner, spheres, polygons, NULL, distance_metric, &o_type);
                PrQueueObstacle *new_po = malloc(sizeof(PrQueueObstacle));
                new_po->data = new_blocking;
                if(new_blocking != p && new_blocking != from->obstacle && !prqueue_contains(local, new_po)) {
                  po->type = o_type;
                  prqueue_add(local, new_po);
                }
              }
            }

          }
        }
      }
    }

    return g;
  
    
  } else {
    // This was done by me at work, I forgot to push to the repo ...
  }

  return g;

}


MapPoint *vstar(MapPoint *start, MapPoint *goal, DList *polygons, DList *spheres, double (*distance_metric)(void*, void*), short VERBOSE) {

  UNUSED(spheres);
  UNUSED(VERBOSE);

  HSet *initialized = hset_init(obstacle_hash, equals);

  // =========================================================

  PrQueue *global = prqueue_init(compare_to);
  global->equals = equals;
  prqueue_add(global,start);

  double shortest_to_goal = 9999999999.9l;

  goal->shortest_ancestor = NULL;

  MapPoint *from, *to;
  start->shortest_length = 0.0l;
  start->shortest_ancestor = NULL;
  start->local_queue = prqueue_init(compare_to);
  start->visited_points = dlist_init();

  for(size_t ii = 0; ii < polygons->num_items; ii++) {
    PolygonalObstacle *p = darray_get(polygons->data,ii);
    for(size_t jj = 0; jj < p->corners->num_items; jj++) {
      MapPoint *m = darray_get(p->corners->data, jj);
      m->local_queue = prqueue_init(compare_to);
      m->visited_points = dlist_init();
      m->shortest_length = 9999999999.9l;
      m->shortest_ancestor = NULL;
    }
  }

  // Get the next item on the todo list
  while((from = prqueue_pop(global)) != NULL) {

    if(VERBOSE == 2) {
      fprintf(stderr, "%lu\n", global->num_items);
    }

    if(hset_contains(initialized, from) == -1) {

      // Initialize the relevant parts of the MapPoint struct
      if(from != goal) {
        prqueue_add(from->local_queue, goal);
      }

      // If the point is the corner of a polygon (very likely),
      // add its neighbors right away
      PolygonalObstacle *p = from->obstacle;

      if(p != NULL) {
        long tmp_idx = darray_find(p->corners->data, from);
        size_t idx = (tmp_idx > 0) ? tmp_idx : 0;
        MapPoint *n;

        double d;

        n = darray_get(p->corners->data, (idx+1) % p->corners->num_items);
        d = distance_metric(from,n);
        if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
          n->shortest_ancestor = from;
          n->shortest_length = d;
          prqueue_add(n->local_queue, from->shortest_ancestor);
          prqueue_add(global, n);
        }

        n = darray_get(p->corners->data, (idx > 0) ? idx - 1 : p->corners->num_items-1);
        d = distance_metric(from,n);
        if(from->shortest_length + d < n->shortest_length && from->shortest_length + d < shortest_to_goal) {
          n->shortest_ancestor = from;
          n->shortest_length = d;
          prqueue_add(n->local_queue, from->shortest_ancestor);
          prqueue_add(global, n);
        }
      }

      // Done ;)
      hset_add(initialized, from);
    }

    if(VERBOSE >= 2) {
      fprintf(stderr, "Checking (%g,%g) -> %lu nodes to inspect\n", from->x, from->y, from->local_queue->num_items);
    }

    if(from == goal || from->local_queue->num_items == 0) continue;

    // The point may have some points listed that it should try to reach.
    while((to = prqueue_pop(from->local_queue)) != NULL) {

      if(from->obstacle && from->obstacle == to->obstacle) continue;

      if(VERBOSE >= 2) {
        fprintf(stderr, "    Trying to reach (%g,%g)\n", to->x, to->y);
      }

      if(darray_find(from->visited_points->data, to) > -1) {
        continue;
      }
      dlist_push(from->visited_points, to);

      // As always, try to make a direct connection first

      if(!polygon_is_blocked(from, to, polygons)) {
        if(VERBOSE >= 2) {
          fprintf(stderr, "        Connection is possible!\n");
        }

        double d = distance_metric(from,to);
        if(from->shortest_length + d < to->shortest_length && from->shortest_length + d < shortest_to_goal) {
          // Update forward
          to->shortest_length = from->shortest_length + d;
          to->shortest_ancestor = from;
          if(to == goal) shortest_to_goal = from->shortest_length + d;

          // Update backward
          if(from->shortest_ancestor) {
            prqueue_add(from->shortest_ancestor->local_queue, to);
            prqueue_add(global, from->shortest_ancestor);
          }

          prqueue_add(global, to);
         }

      } else {

        PolygonalObstacle *p = polygon_get_first_blocking(from, to, polygons, from->obstacle);

        if(VERBOSE >= 2) {
          if(p) {
            fprintf(stderr, "        Blocked by %p \n", (void*) p);
            double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
            double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
            fprintf(stderr, "              (%g,%g)\n", p_x, p_y);
          }
        }

        // Do the whole expansion process as above in the other method, starting from the
        // current point
        PrQueue *local = prqueue_init(compare_to);
        HSet *local_explored = hset_init(obstacle_hash, equals);
        local->equals = equals;
        prqueue_add(local, p);
        while((p = prqueue_pop(local)) != NULL) {

          if(hset_contains(local_explored, p) != -1) {
            continue;
          } else {
            hset_add(local_explored, p);
          }

          if(VERBOSE >= 2) {
            double p_x = ((MapPoint*) darray_get(p->corners->data, 0))->x;
            double p_y = ((MapPoint*) darray_get(p->corners->data, 0))->y;
            fprintf(stderr, "       Current obstacle: (%g,%g)\n", p_x, p_y);
          }

    
          if(p == from->obstacle) continue;

          for(size_t ii = 0; ii < p->corners->num_items; ii++) {

            MapPoint *corner = darray_get(p->corners->data, ii);

            if(VERBOSE >= 2) {
              fprintf(stderr, "      Trying to reach (%g,%g)\n", corner->x, corner->y);
            }

            if(!polygon_is_blocked(from, corner, polygons)) {
              double d = distance_metric(from,corner);
              if(from->shortest_length + d < corner->shortest_length && from->shortest_length + d < shortest_to_goal) {
                // Update forward
                corner->shortest_length = from->shortest_length + d;
                corner->shortest_ancestor = from;
                if(corner == goal) shortest_to_goal = from->shortest_length + d;

                // Update backward
                if(from->shortest_ancestor) {
                  prqueue_add(from->shortest_ancestor->local_queue, corner);
                  prqueue_add(global, from->shortest_ancestor);
                }

                prqueue_add(global, corner);

              }
            } else {
              PolygonalObstacle *block = polygon_get_first_blocking(from, corner, polygons, from->obstacle);
              if(block != p && block != from->obstacle && !prqueue_contains(local, block)) {
                prqueue_add(local, block);
              }
            }
          }
        }
      }
    }
  }

  return NULL;
}

void* get_first_blocking(MapPoint *from, MapPoint *to, DList *spheres, DList *polygons, void *self, double (*distance_metric)(void*,void*), enum OBSTACLE_TYPES* type) {

  PolygonalObstacle *p = polygon_get_first_blocking(from, to, polygons, self);
  CircularObstacle *c = tangent_get_first_blocking(from, to, spheres, self, distance_metric);

  if(p == NULL) {
    *type = SPHERICAL_OBSTACLE;
    return c;
  }

  if(c == NULL) {
    *type = POLYGONAL_OBSTACLE;
    return p;
  }

  double r = 1.0;
  double r_min = r;
  for(size_t idx = 0; idx < p->corners->num_items; idx++) {
    MapPoint *v1 = darray_get(p->corners->data, idx);
    MapPoint *v2 = darray_get(p->corners->data, (idx+1) % p->corners->num_items);
    
    _lines_intersect(from, to, v1, v2, &r, NULL);
    r_min = (r < r_min) ? r : r_min;
  }

  double dx = to->x - from->x;
  double dy = to->y - from->y;

  double _a = sq(dx) + sq(dy);
  double _b = 2.0 * (from->x * dx - dx * c->position.x + from->y * dy - dy * c->position.y);
  double _c = sq(from->x - c->position.x) + sq(from->y - c->position.y) - sq(c->radius);

  double t1 = -_b / (2.0 * _a) + sqrt((sq(_b) - 4.0 * _c * _a) / (4.0 * sq(_a)));
  double t2 = -_b / (2.0 * _a) - sqrt((sq(_b) - 4.0 * _c * _a) / (4.0 * sq(_a)));

  double t_min = (t1 < t2) ? t1 : t2;

  if(t_min < r_min) {
    *type = SPHERICAL_OBSTACLE;
    return (void*) c;
  } else {
    *type = POLYGONAL_OBSTACLE;
    return (void*) p;
  }
}

unsigned short is_blocked(MapPoint *from, MapPoint *to, DList *spheres, DList *polygons) {

  return polygon_is_blocked(from, to, polygons) + tangent_is_blocked(from, to, spheres);

}
