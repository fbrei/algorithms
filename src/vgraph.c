#include "include/vgraph.h"
#include "include/globals.h"
#include <stdio.h>

#define PRINTDEBUG 0

int compare_to(void* a,void* b) {
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

void print_polygon(FILE *stream, void *o) {
  PolygonalObstacle *p = (PolygonalObstacle*) o;
  for(size_t idx = 0; idx < p->corners->num_items; idx++) {
    MapPoint *c = darray_get(p->corners->data,idx);
    fprintf(stream, "(%g,%g) -> ", c->x, c->y);
  }
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
      fprintf(stderr, "Expanding %p : ", current);
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
          PolygonalObstacle *p = polygon_get_first_blocking(start,corner,obstacles,o);
          if(p == NULL) {
            graph_connect(g,start,corner,distance_metric(start,corner));
          } else {
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

        // Initialize local queue by trying to reach the goal directly
        size_t n_corners = o->corners->num_items;
        for(size_t ii = 0; ii < n_corners; ii++) {
          MapPoint *corner = darray_get(o->corners->data,ii);


          if(!polygon_is_blocked(corner,goal,obstacles)) {
            graph_connect(g,corner,goal,distance_metric(corner,goal));
          } else {
            PolygonalObstacle *p = polygon_get_first_blocking(corner,goal,obstacles,o);
            if(!prqueue_contains(local,p)) {
              prqueue_add(local,p);
            }
          }
        }

        // The Q has been prepared, now we can start
        // working on it
        PolygonalObstacle *p;
        while((p = prqueue_pop(local)) != NULL) {
          if(hset_contains(local_explored,p) != -1) {
            continue;
          } else {
            hset_add(local_explored, p);
          }

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
      for(int jj = 0; jj < n_p_corners; jj++) {
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

Graph* vgraph(MapPoint *start, MapPoint *goal, DList *polygons, DList *spheres, double (*distance_metric)(void*, void*), const int dynamic) {

  Graph *g = graph_init(GRAPH_DIRECTED);

  if(dynamic == 2) {
    
  } else {
    
    // We will first call the full graph method for spherical obstacles to 
    // get all tangent points. This is computationally expensive on a
    // ridiculous level but it suffices for now
    
    g = vgraph_circular_obstacles(start, goal, spheres, distance_metric, 0);
    size_t n_original_nodes = g->node_list->num_items;

    for(size_t ii = 0; ii < polygons->data->num_items; ii++) {
      PolygonalObstacle *p = darray_get(polygons->data,ii);
      for(size_t jj = 0; jj < p->corners->num_items; jj++) {
        graph_add(g,darray_get(p->corners->data,jj));
      }
    }

    for(size_t ii = 0; ii < n_original_nodes; ii++) {

      MapPoint *m = darray_get(g->node_list, ii);

      for(size_t jj = 0; jj < polygons->num_items; jj++) {

        PolygonalObstacle *p = darray_get(polygons->data,jj);

        for(size_t kk = 0; kk < p->corners->num_items; kk++) {

          MapPoint *n = darray_get(p->corners->data,kk);

          if(!polygon_is_blocked(m,n,polygons)) {
           graph_connect(g,n,m,distance_metric(m,n));
           graph_connect(g,m,n,distance_metric(m,n));
          }

        }
      }
    }
  }

  return g;

}
