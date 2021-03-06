#include "include/polygons.h"

#include <stdio.h>
#include <math.h>
#define M_PI 3.1415926535897932

unsigned short _lines_intersect(MapPoint *from_a, MapPoint *to_a, MapPoint *from_b, MapPoint *to_b, double *_r, double *_s) {

  // First line: A. First coordinates (x0,y0), go (x1,y1)
  // lines parametric: (x0,y0) + r*(dx,dy), where r is in [0,1]
  
  double x0_a = from_a->x,
         y0_a = from_a->y,
         x1_a = to_a->x,
         y1_a = to_a->y,

         x0_b = from_b->x,
         y0_b = from_b->y,
         x1_b = to_b->x,
         y1_b = to_b->y;
         ;

   if(x0_a == x1_a) {
     if(x0_b == x1_b) {
       if(x0_b == x0_a) {
         return 1;
       } else {
         return 0;
       }
     }

     double s = (x0_a - x0_b) / (x1_b - x0_b);
     if(s < 0 || s > 1) return 0;

     double y = y0_b + s * y1_b;
     if(y >= y0_a && y <= y1_a) {
       return 1;
     } else {
       return 0;
     }
   }
   double r = y0_b * (x1_b - x0_b) + (y1_b - y0_b) * (x0_a - x0_b) - y0_a * (x1_b - x0_b);
   r /= ((x1_b - x0_b) * (y1_a - y0_a) - (x1_a - x0_a) * (y1_b - y0_b));

   if(r < 0 || r > 1) return 0;

   if(x1_b == x0_b) {
     double y = y0_a + r * (y1_a - y0_a);
     double y_start = (y0_b > y1_b) ? y1_b : y0_b;
     double y_end = (y0_b < y1_b) ? y1_b : y0_b;
     if(y >= y_start && y <= y_end) {
       return 1;
     } else {
       return 0;
     }
   }
   double s =  ((x0_a - x0_b) + r * (x1_a - x0_a)) / (x1_b - x0_b);

   if(s < 0 || s > 1) return 0;

   if(_r) *_r = r;
   if(_s) *_s = s;
   return 1;
}

void polygon_destroy(PolygonalObstacle *po) {
  dlist_destroy(po->corners, NULL);
  free(po);
}

unsigned short polygon_is_blocked(MapPoint *p1, MapPoint *p2, DList *obstacles) {

  double dx_p = p2->x - p1->x;
  double dy_p = p2->y - p1->y;

  for(size_t ii = 0; ii < obstacles->num_items; ii++) {
    PolygonalObstacle *p = darray_get(obstacles->data, ii);
    size_t n_corners = p->corners->num_items;
    for(size_t jj = 0; jj < n_corners; jj++) {
      MapPoint *o1 = darray_get(p->corners->data, jj);
      MapPoint *o2 = darray_get(p->corners->data, (jj+1) % n_corners);
      if(o1 == p1 || o1 == p2 || o2 == p1 || o2 == p2) {
        continue;
      }

      double dx_o = o2->x - o1->x;
      double dy_o = o2->y - o1->y;

      double t = (dy_p * p1->x - dx_p * p1->y - dy_p * o1->x + dx_p * o1->y) / (dy_p * dx_o - dx_p * dy_o);

      if(t >= 0 && t <= 1) {
        double s = ((o1->x-p1->x) + (o1->y-p1->y) + t * (dx_o + dy_o)) / (dx_p + dy_p);
        if(s >= 0 && s <= 1) {
          return 1;
        }
      }
    }
  }

  return 0;
}

PolygonalObstacle* polygon_get_first_blocking(MapPoint* from, MapPoint* to, DList* obstacles, PolygonalObstacle* self) {
  double dx_p = to->x - from->x;
  double dy_p = to->y - from->y;

  double best_s = 1.0;
  PolygonalObstacle* closest_obstacle = NULL;

  size_t n_obstacles = obstacles->num_items;
  for(size_t idx = 0; idx < n_obstacles; idx++) {
    PolygonalObstacle* current = darray_get(obstacles->data, idx);
    if(current == self) {
      continue;
    }

    size_t n_corners = current->corners->num_items;
    for(size_t inner_idx = 0; inner_idx < n_corners; inner_idx++) {
      MapPoint *o1 = darray_get(current->corners->data, inner_idx);
      MapPoint *o2 = darray_get(current->corners->data, (inner_idx+1) % n_corners);

      double dx_o = o2->x - o1->x;
      double dy_o = o2->y - o1->y;

      double t = (dy_p * from->x - dx_p * from->y - dy_p * o1->x + dx_p * o1->y) / (dy_p * dx_o - dx_p * dy_o);
      double s = ((o1->x-from->x) + (o1->y-from->y) + t * (dx_o + dy_o)) / (dx_p + dy_p);

      if(s < best_s && s > 0 && t > 0 && t < 1) {
        best_s = s;
        closest_obstacle = current;
      }
    }
  }

  return closest_obstacle;
}

PolygonalObstacle* convex_hull(DList* map_points) {

  size_t n_points = map_points->num_items;

  // Graham Scan
  // Step 1 - find the left most point
  MapPoint *p0 = darray_get(map_points->data,0);

  for(size_t idx = 1; idx < n_points; idx++) {
    MapPoint *p = darray_get(map_points->data,idx);

    if( (p->x < p0->x) || (p->x == p0->x && p->y < p0->y) ) {
      p0 = p;
    }
  }

  // Step 2 - Calculate all angles of the vectors starting at P0
  // and ending at any other point
  double *scores = malloc(sizeof(double) * n_points);

  for(size_t idx = 0; idx < n_points; idx++) {
    MapPoint *p = darray_get(map_points->data,idx);
    double dx = p->x - p0->x;
    double dy = p->y - p0->y;

    if(p == p0) {
      scores[idx] = -M_PI / 2.0;
    } else {
      scores[idx] = (dx > 0) ? atan(dy / dx) : (M_PI / 2.0);
    }
  }

  // Step 3 - Sort the points according to their scores
  // We use bubble sort because it is fastest for small
  // instances (< 20 on test machine)

  MapPoint *tmp_point;
  double tmp_score;
  for(size_t ii = 1; ii < n_points; ii++) {
    for(size_t jj = 0; jj < n_points-ii; jj++) {
      if(scores[jj] > scores[jj+1]) {
        tmp_point = darray_get(map_points->data,jj);
        darray_set(map_points->data, darray_get(map_points->data,jj+1),jj);
        darray_set(map_points->data,tmp_point,jj+1);

        tmp_score = scores[jj];
        scores[jj] = scores[jj+1];
        scores[jj+1] = tmp_score;
      }
    }
  }


  // Step 4 - remove all points with equal score, keeping only
  // those that are further away from P0

  DList *hull_candidates = dlist_init();
  /* double last_score = -9999.9; */
  /* size_t hull_idx = 0; */
  for(size_t idx = 0; idx < n_points; idx++) {
    dlist_push(hull_candidates, darray_get(map_points->data,idx));
    /* if(scores[idx] == last_score) { */
    /*   MapPoint *p1 = darray_get(hull_candidates->data,hull_idx-1); */
    /*   MapPoint *p2 = darray_get(map_points->data,idx); */
    /*  */
    /*   double dx1 = p1->x - p0->x; */
    /*   double dy1 = p1->y - p0->y; */
    /*  */
    /*   double dx2 = p2->x - p0->x; */
    /*   double dy2 = p2->y - p0->y; */
    /*  */
    /*   double dist_1 = dx1 * dx1 + dy1 * dy1; */
    /*   double dist_2 = dx2 * dx2 + dy2 * dy2; */
    /*  */
    /*   if(dist_2 < dist_1) { */
    /*     darray_set(hull_candidates->data,p2,hull_idx-1); */
    /*   } */
    /* } else { */
    /*   dlist_push(hull_candidates, darray_get(map_points->data,idx)); */
    /*   last_score = scores[idx]; */
    /*   hull_idx++; */
    /* } */
  }

  // Step 5 - Now actually create the hull
  DList *hull = hull_candidates;
  size_t idx = 1;
  size_t l = hull->num_items;

  while(idx < l) {
    MapPoint *start = darray_get(hull->data, idx - 1);
    MapPoint *end = darray_get(hull->data, (idx+1) % l);
    MapPoint *inter = darray_get(hull->data, idx);

    // The determinant will tell us if the triangle is clockwise
    // or counter-clockwise oriented (cw - keep, ccw - discard)
    double det = (inter->x - start->x) * (end->y - start->y) - (end->x - start->x) * (inter->y - start->y);
    if(det < 0) {
      // Triangle is turning 'outwards', this means that the third point lies
      // outside the current hull and therefore the intermediate point can be
      // discarded
      DList *new_hull = dlist_init();
      for(size_t ii = 0; ii < l; ii++) {
        if(ii == idx) continue;
        dlist_push(new_hull, darray_get(hull->data,ii));
      }
      dlist_destroy(hull,NULL);
      hull = new_hull;
      l = hull->num_items;
      idx = (idx > 0) ? idx - 1 : l - 1;
    } else {
      idx++;
    }
  }


  PolygonalObstacle *p = malloc(sizeof(PolygonalObstacle));
  p->corners = hull;
  for(size_t ii = 0; ii < hull->num_items; ii++) {
    ((MapPoint*) darray_get(hull->data, ii))->obstacle = p;
  }

  return p;

}

double polygon_area(PolygonalObstacle *p) {
  
  double total_area = 0.0l;
  for(size_t ii = 0; ii < p->corners->num_items; ii++) {
    MapPoint *from = darray_get(p->corners->data, ii);
    MapPoint *to = darray_get(p->corners->data, (ii+1) % p->corners->num_items);

    total_area += (from->x * to->y - from->y * to->x);
  }

  return fabs(total_area) / 2.0;
}

double polygon_map_covered(DList *polygons, double width, double height) {

  double occupied = 0.0l;
  for(size_t idx = 0; idx < polygons->num_items; idx++) {
    occupied += polygon_area(darray_get(polygons->data, idx));
  }

  return occupied / (width * height);
}

unsigned short polygon_overlapping(PolygonalObstacle *p, PolygonalObstacle *o) {

  // Use ray casting. We need the bounding box of both polygons
  // for this because we need to find a point that is definitely outside.
  MapPoint *m; 
  double p_x_min, p_y_min, o_x_min, o_y_min;
  
  m = darray_get(p->corners->data, 0);
  p_x_min = m->x;
  p_y_min = m->y;

  m = darray_get(o->corners->data, 0);
  o_x_min = m->x;
  o_y_min = m->y;

  for(size_t idx = 1; idx < p->corners->num_items; idx++) {
    m = darray_get(p->corners->data, idx);
    p_x_min = (m->x < p_x_min) ? m->x : p_x_min; 
    p_y_min = (m->y < p_y_min) ? m->y : p_y_min; 
  }

  for(size_t idx = 1; idx < o->corners->num_items; idx++) {
    m = darray_get(o->corners->data, idx);
    o_x_min = (m->x < o_x_min) ? m->x : o_x_min; 
    o_y_min = (m->y < o_y_min) ? m->y : o_y_min; 
  }

  // Now, for both polygons: choose a point outside the bounding box,
  // cast a ray from it to each corner and count for each ray how
  // often it intersects with an edge of the other polygon. If for one
  // ray there is an odd number of intersections, than the target point
  // is within the other polygon
  
  // Start with bounding box of P
  MapPoint *from = malloc(sizeof(MapPoint)), *to;
  from->x = o_x_min - 1.0, from->y = o_y_min - 1.0;

  for(size_t idx = 0; idx < p->corners->num_items; idx++) {
    to = darray_get(p->corners->data, idx);
    size_t count = 0;
    for(size_t jj = 0; jj < o->corners->num_items; jj++) {
      MapPoint *edge_from = darray_get(o->corners->data, jj);
      MapPoint *edge_to = darray_get(o->corners->data, (jj+1) % o->corners->num_items);
      count += _lines_intersect(from, to, edge_from, edge_to, NULL, NULL);
    }
    if(count % 2 == 1) { return 1; }
  }

  // Now the other one
  from->x = p_x_min - 1.0, from->y = p_y_min - 1.0;

  for(size_t idx = 0; idx < o->corners->num_items; idx++) {
    to = darray_get(o->corners->data, idx);
    size_t count = 0;
    for(size_t jj = 0; jj < p->corners->num_items; jj++) {
      MapPoint *edge_from = darray_get(p->corners->data, jj);
      MapPoint *edge_to = darray_get(p->corners->data, (jj+1) % p->corners->num_items);
      count += _lines_intersect(from, to, edge_from, edge_to, NULL,NULL);
    }
    if(count % 2 == 1) { return 1; }
  }

  // Now check if any two lines intersect (expensive, that's why we do it last)
  
  for(size_t ii = 0; ii < p->corners->num_items; ii++) {
      MapPoint *from_p = darray_get(p->corners->data, ii);
      MapPoint *to_p = darray_get(p->corners->data, (ii+1) % p->corners->num_items);

      for(size_t jj = 0; jj < o->corners->num_items; jj++) {
        MapPoint *from_o = darray_get(o->corners->data, jj);
        MapPoint *to_o = darray_get(o->corners->data, (jj+1) % o->corners->num_items);

        if(_lines_intersect(from_p, to_p, from_o, to_o,NULL,NULL)) return 1;
      }
  }

  return 0;
}
