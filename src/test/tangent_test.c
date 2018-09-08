#include "tangents.h"
#include "dtype.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

void print_results(DArray *out) {
  MapPoint *tmp = NULL;
  while(1) {
    tmp = (MapPoint*) darray_iterate(out, tmp);
    if(tmp == NULL) {
      break;
    }
    printf("(%5.2lf, %5.2lf) -> ", tmp->x, tmp->y);
    tmp = (MapPoint*) darray_iterate(out, tmp);
    printf("(%5.2lf, %5.2lf)\n", tmp->x, tmp->y);
  }
}

void print_point(MapPoint m) {
  printf("(%g,%g)", m.x, m.y);
}

void example1() {

  printf("Tangent circle point\n");
  printf("====================\n");

  MapPoint p = { .x = -6, .y = -8 };
  CircularObstacle c = { .radius = 2, .position = { .x = -3, .y = -5} };

  DArray *out = tangent_circle_point_intersects(&p, &c);

  if(out) {

    print_results(out);
    darray_destroy(out,free);
  }

  printf("\n");

}

void example2() {

  printf("Tangent circle circle\n");
  printf("=====================\n");

  double m_x = -3.0, m_y = -5.0, m_r = 2.0;
  double n_x = 10.0, n_y = 3.0, n_r = 2.0;

  printf("(x-%g)^2 + (y-%g)^2 = %g^2\n",m_x,m_y,m_r);
  printf("(x-%g)^2 + (y-%g)^2 = %g^2\n",n_x,n_y,n_r);

  CircularObstacle c1 = { .radius = m_r, .position = { .x = m_x, .y = m_y} };
  CircularObstacle c2 = { .radius = n_r, .position = { .x = n_x, .y = n_y} };

  printf("Outer tangent points:\n");
  DArray *out = tangent_circle_outer_intersects(&c1, &c2);

  if(out) {

    print_results(out);
    darray_destroy(out,free);
  }

  printf("Inner tangent points:\n");
  out = tangent_circle_inner_intersects(&c1, &c2);

  if(out) {

    print_results(out);
    darray_destroy(out,free);
  }
}

void example3() {

  printf("Filtered tangents between two circles\n");
  printf("=====================================\n");

  double m_x = 10.0, m_y = 3.0, m_r = 2.0;
  double n_x = -3.0, n_y = -5.0, n_r = 2.0;

  double o_x = 3.0, o_y = 2.0, o_r = 2.0;

  CircularObstacle co = { .radius = o_r, .position = { .x = o_x, .y = o_y } };

  DArray *obstacles = darray_init();
  darray_set(obstacles, &co, 0);

  printf("(x-%g)^2 + (y-%g)^2 = %g^2\n",m_x,m_y,m_r);
  printf("(x-%g)^2 + (y-%g)^2 = %g^2\n",n_x,n_y,n_r);

  CircularObstacle c1 = { .radius = m_r, .position = { .x = m_x, .y = m_y} };
  CircularObstacle c2 = { .radius = n_r, .position = { .x = n_x, .y = n_y} };

  DArray *out = tangent_circle_outer_intersects(&c1, &c2);

  MapPoint *tmp = NULL;
  if(out) {

    clock_t start, end;
    start = clock();
    while(1) {
      tmp = (MapPoint*) darray_iterate(out, tmp);

      if(tmp == NULL) break;

      MapPoint *first = tmp;

      print_point(*first);

      tmp = (MapPoint*) darray_iterate(out, tmp);
      MapPoint *second = tmp;

      printf(" -> ");
      print_point(*second);

      if(tangent_is_blocked(first, second, obstacles)) {
        printf(" !! Path is blocked !!");
      }
      printf("\n");
    }
    end = clock();
    double time_taken = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Checking all routes took %g seconds\n", time_taken);
     
    darray_destroy(out,free);
  }

  darray_destroy(obstacles, NULL);

}

void example4() {

  printf("Calculate an entire graph with start and goal\n");
  printf("=============================================\n");
  printf("For a visualization visit: \n");
  printf("https://www.desmos.com/calculator/ldnhvjpg9q\n");
  printf("\n");

  MapPoint start = { .x = 14, .y = 8 }, goal = { .x = -6, .y = -8 };

  DArray *obstacles = darray_init();

  CircularObstacle c1 = { .position = { .x = 10, .y = 3}, .radius = 2 };
  darray_set(obstacles, &c1, 0);

  CircularObstacle c2 = { .position = { .x = 3, .y = 2}, .radius = 2 };
  darray_set(obstacles, &c2, 1);

  CircularObstacle c3 = { .position = { .x = -3, .y = -5}, .radius = 2 };
  darray_set(obstacles, &c3, 2);


  // What follows is basically the turn_map_into_graph function
  CircularObstacle *tmp_obstacle = NULL;
  DArray *out = NULL;

  while((tmp_obstacle = (CircularObstacle*) darray_iterate(obstacles, tmp_obstacle)) != NULL) {

    out = tangent_circle_point_intersects(&start, tmp_obstacle);
    MapPoint *tmp_point = NULL;
    while((tmp_point = (MapPoint*) darray_iterate(out, tmp_point)) != NULL) {
      if(tangent_is_blocked(&start, tmp_point, obstacles)) continue;
      print_point(start);
      printf(" -> ");
      print_point(*tmp_point);
      printf("\n");
    }
    darray_destroy(out, free);


    out = tangent_circle_point_intersects(&goal, tmp_obstacle);
    tmp_point = NULL;
    while((tmp_point = (MapPoint*) darray_iterate(out, tmp_point)) != NULL) {
      if(tangent_is_blocked(&goal, tmp_point, obstacles)) {
        continue;
      }
      print_point(goal);
      printf(" -> ");
      print_point(*tmp_point);
      printf("\n");
    }
    darray_destroy(out, free);


    CircularObstacle *other_obstacle = NULL;

    while((other_obstacle = (CircularObstacle*) darray_iterate(obstacles, other_obstacle)) != NULL) {
      if(other_obstacle == tmp_obstacle) continue;

      out = tangent_circle_inner_intersects(tmp_obstacle, other_obstacle);

      MapPoint *first = NULL, *second = NULL;

      while(1) {
        first = darray_iterate(out, second);
        if(first == NULL) break;
        second = darray_iterate(out, first);

        if(tangent_is_blocked(first,second,obstacles)) {
          continue;
        }
        print_point(*first);
        printf(" -> ");
        print_point(*second);
        printf("\n");
      }

      darray_destroy(out, free);
    }

  }

  darray_destroy(obstacles, NULL);
}

int main(int argc, const char** argv) {

  example4();

  return 0;
}
