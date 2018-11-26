#include "include/polygons.h"

#include <stdio.h>

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
      double s = ((o1->x-p1->x) + (o1->y-p1->y) + t * (dx_o + dy_o)) / (dx_p + dy_p);

      if(t >= 0 && t <= 1) {
        if(s >= 0 && s <= 1) {
          return 1;
        }
      }
    }
  }

  return 0;
}

PolygonalObstacle* polygon_get_first_blocking(MapPoint* from, MapPoint* to, DList* obstacles, PolygonalObstacle* self) {
  /* MOVE ME TO SOURCE FILE */
}
