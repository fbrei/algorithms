#include "tangents.h"
#include "dtype.h"
#include "astar.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define PRINT_GRAPH 0

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

void print_graph_point(void *v) {
  MapPoint *m = (MapPoint*) v;
  printf("(%g,%g)", m->x, m->y);
}

double heuristic(MapPoint *m1, MapPoint *m2) {
  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);
}

double heuristic_astar(void *n1, void *n2) {

  MapPoint *m1 = (MapPoint*) n1;
  MapPoint *m2 = (MapPoint*) n2;

  double dx = m1->x - m2->x;
  double dy = m1->y - m2->y;

  return sqrt(dx*dx + dy*dy);
}

void print_obstacle(void *o) {

  CircularObstacle *co = (CircularObstacle*) o;

  printf("Obstacle: (%g,%g)\n",co->position.x, co->position.y);
  void *tmp = NULL;
  while((tmp = darray_iterate(co->_map_points, tmp)) != NULL) {
    MapPoint *tmp_point = (MapPoint*) tmp;
    printf("(%g,%g)\n",tmp_point->x, tmp_point->y);
  }

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

  // This function suffers from heavy memory leakage but I will 
  // not fix this because this is just an example and a testing
  // ground for the actual function that I implemented after
  // this one

  printf("Calculate an entire graph with start and goal\n");
  printf("=============================================\n");
  printf("For a visualization visit: \n");
  printf("https://www.desmos.com/calculator/ldnhvjpg9q\n");
  printf("\n");

  MapPoint start = { .x = 14, .y = 8 }, goal = { .x = -6, .y = -8 };

  DArray *obstacles = darray_init();

  CircularObstacle *c1 =  obstacle_init(10,3,2);
  darray_set(obstacles, c1, 0);

  CircularObstacle *c2 =  obstacle_init(3,2,2);
  darray_set(obstacles, c2, 1);

  CircularObstacle *c3 =  obstacle_init(-3,-5,2);
  darray_set(obstacles, c3, 2);


  // What follows is basically the turn_map_into_graph function
  CircularObstacle *tmp_obstacle = NULL;
  DArray *out = NULL;

  clock_t start_time, end_time;

  start_time = clock();

  Graph *g = graph_init(GRAPH_DIRECTED);
  graph_add(g, &start);
  graph_add(g, &goal);

  while((tmp_obstacle = (CircularObstacle*) darray_iterate(obstacles, tmp_obstacle)) != NULL) {

    out = tangent_circle_point_intersects(&start, tmp_obstacle);
    MapPoint *tmp_point = NULL;
    while((tmp_point = (MapPoint*) darray_iterate(out, tmp_point)) != NULL) {
      if(tangent_is_blocked(&start, tmp_point, obstacles)) continue;

      _obstacle_add_map_point(tmp_obstacle, tmp_point);
      graph_add(g, tmp_point);
      graph_connect(g, &start, tmp_point, heuristic(&start, tmp_point));
#if PRINT_GRAPH
      print_point(start);
      printf("\n");
      /* printf(" -> "); */
      print_point(*tmp_point);
      printf("\n");
#endif
    }


    out = tangent_circle_point_intersects(&goal, tmp_obstacle);
    tmp_point = NULL;
    while((tmp_point = (MapPoint*) darray_iterate(out, tmp_point)) != NULL) {
      if(tangent_is_blocked(&goal, tmp_point, obstacles)) {
        continue;
      }
      _obstacle_add_map_point(tmp_obstacle, tmp_point);
      graph_add(g, tmp_point);
      graph_connect(g, tmp_point, &goal, heuristic(&goal, tmp_point));
#if PRINT_GRAPH
      print_point(goal);
      printf("\n");
      /* printf(" -> "); */
      print_point(*tmp_point);
      printf("\n");
#endif
    }


    CircularObstacle *other_obstacle = NULL;

    while((other_obstacle = (CircularObstacle*) darray_iterate(obstacles, other_obstacle)) != NULL) {
      if(other_obstacle == tmp_obstacle) break;

      out = tangent_circle_inner_intersects(tmp_obstacle, other_obstacle);

      MapPoint *first = NULL, *second = NULL;

      while(1) {
        first = darray_iterate(out, second);
        if(first == NULL) break;
        second = darray_iterate(out, first);

        if(tangent_is_blocked(first,second,obstacles)) {
          continue;
        }

        graph_add(g, first);
        graph_add(g, second);
        graph_connect(g, second, first, heuristic(first, second));

        _obstacle_add_map_point(tmp_obstacle, first);
        _obstacle_add_map_point(other_obstacle, second);

#if PRINT_GRAPH
        print_point(*first);
        printf("\n");
        /* printf(" -> "); */
        print_point(*second);
        printf("\n");
#endif
      }


      out = tangent_circle_outer_intersects(tmp_obstacle, other_obstacle);

      first = NULL, second = NULL;

      while(1) {
        first = darray_iterate(out, second);
        if(first == NULL) break;
        second = darray_iterate(out, first);

        if(tangent_is_blocked(first,second,obstacles)) {
          continue;

        }

        graph_add(g, first);
        graph_add(g, second);
        graph_connect(g, second, first, heuristic(first, second));

        _obstacle_add_map_point(tmp_obstacle, first);
        _obstacle_add_map_point(other_obstacle, second);
#if PRINT_GRAPH
        print_point(*first);
        printf("\n");
        /* printf(" -> "); */
        print_point(*second);
        printf("\n");
#endif
      }


    }

  }

  void *tmp = NULL;
  while((tmp = darray_iterate(obstacles, tmp)) != NULL) {
    _obstacle_connect_map_points((CircularObstacle*) tmp, g);
  }

#if PRINT_GRAPH
  graph_print(g, print_graph_point);
#endif


  AStarPathNode *p = astar(g, &start, &goal, heuristic_astar, NULL);

  end_time = clock();
  double time_taken = ((double) end_time - start_time) / CLOCKS_PER_SEC;
  printf("Elapsed time: %lfs\n", time_taken);

  while(p) {
    MapPoint *m = p->data;
    print_point(*m);
    printf("\n");
    p = p->parent;
  }

  /* print_obstacle(c1); */

  graph_destroy(g,NULL);

  darray_destroy(obstacles, NULL);
}

int main(int argc, const char** argv) {

  example4();

  return 0;
}
