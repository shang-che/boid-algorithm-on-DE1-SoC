#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
// for keyboard
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define M_PI 3.14159265358979323846
// f (focal length)是視場深度(焦距)
// d 是攝影機到投影平面的距離
// 我們假設 f = 1 和 d=3。
// #define f 1
// #define d 3
#define X_MIN -320
#define X_MAX 320
#define Y_MIN -240
#define Y_MAX 240


typedef struct {
  double x, y, z;
} Position_3d;

typedef struct {
  double x, y;
} Projection_3d;

// boid function
typedef struct boid {
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
  double monitor_x;
  double monitor_y;
  double r;
  int color;
} Boid;

// +:put to right -:put to left, b-a:max->min
int compare_boids(const void *a, const void *b) {
  return (((Boid *)b)->z - ((Boid *)a)->z);
}

void rotate_x(Position_3d *point, double theta_x) {
  double y_new = point->y * cos(theta_x) - point->z * sin(theta_x);
  double z_new = point->y * sin(theta_x) + point->z * cos(theta_x);
  point->y = y_new;
  point->z = z_new;
}

void rotate_y(Position_3d *point, double theta_y) {
  double x_new = point->x * cos(theta_y) + point->z * sin(theta_y);
  double z_new = -point->x * sin(theta_y) + point->z * cos(theta_y);
  point->x = x_new;
  point->z = z_new;
}

void rotate_z(Position_3d *point, double theta_z) {
  double x_new = point->x * cos(theta_z) - point->y * sin(theta_z);
  double y_new = point->x * sin(theta_z) + point->y * cos(theta_z);
  point->x = x_new;
  point->y = y_new;
}

// perspective projection
Projection_3d project(Position_3d v, int f, int d) {
  Projection_3d projected_result;
  double x = v.x;
  double y = v.y;
  double z = v.z;
  double x_2D = (x * f) / (z + d);
  double y_2D = (y * f) / (z + d);
  projected_result.x = x_2D;
  projected_result.y = y_2D;
  return projected_result;
}

// 設置標準輸入為非阻塞模式
void set_nonblocking_mode() {
  struct termios ttystate;
  tcgetattr(STDIN_FILENO, &ttystate);
  ttystate.c_lflag &= ~ICANON; // 關閉行緩衝模式
  ttystate.c_lflag &= ~ECHO;   // 關閉回顯
  tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);

  int flags = fcntl(STDIN_FILENO, F_GETFL);
  flags |= O_NONBLOCK; // 設置為非阻塞
  fcntl(STDIN_FILENO, F_SETFL, flags);
}

void projected_zero(Projection_3d A, Projection_3d B) {
  A.x = 0.0;
  A.y = 0.0;
  B.x = 0.0;
  B.y = 0.0;
}

void compute_edge_intersection(Projection_3d display_edge[][2], int in_view_edge[], int i) {
    double t_min = 0.0, t_max = 1.0;
    Projection_3d A, B;
    A = display_edge[i][0];
    B = display_edge[i][1];
    // 計算與垂直邊界的交點
    if (B.x != A.x) {
        double t_x_min = (X_MIN - A.x) / (B.x - A.x);
        double t_x_max = (X_MAX - A.x) / (B.x - A.x);
        t_min = fmax(t_min, fmin(t_x_min, t_x_max));
        t_max = fmin(t_max, fmax(t_x_min, t_x_max));
    }

    // 計算與水平邊界的交點
    if (B.y != A.y) {
        double t_y_min = (Y_MIN - A.y) / (B.y - A.y);
        double t_y_max = (Y_MAX - A.y) / (B.y - A.y);
        t_min = fmax(t_min, fmin(t_y_min, t_y_max));
        t_max = fmin(t_max, fmax(t_y_min, t_y_max));
    }

    // 若 t_min > t_max，表示線段完全在螢幕外，返回無效
    // 我還沒驗證，但應該對?
    if (t_min > t_max) {
      in_view_edge[i] = 0;
      // no display
      projected_zero(A, B);
      printf("edge %d both outside(x,y) and no intersection.\n",i);
      return;
    }
    // A、B要代入哪個座標，1 out or 2 out
    // A is outside
    if (t_min > 0) {
      A = (Projection_3d){
          .x = A.x + t_min * (B.x - A.x),
          .y = A.y + t_min * (B.y - A.y)
      };
      // A.x = A.x + t_min * (B.x - A.x);
      // A.y = A.y + t_min * (B.y - A.y);
    }
    // B is outside
    if (t_max < 1) {
      B = (Projection_3d){
          .x = A.x + t_max * (B.x - A.x),
          .y = A.y + t_max * (B.y - A.y)
      };
      // B.x = A.x + t_max * (B.x - A.x);
      // B.y = A.y + t_max * (B.y - A.y);
    }
    in_view_edge[i] = 1;
    display_edge[i][0] = A;
    display_edge[i][1] = B;
    printf("modified edge %d x:%lf y:%lf , x:%lf y:%lf\n", i, A.x, A.y, B.x, B.y);
}

void determine_in_vision_edge(Projection_3d projected_v[], Projection_3d display_edge[][2], 
                          int edges[][2], int in_view_edge[], int draw_vertices[], int i) {
  display_edge[i][0] = projected_v[edges[i][0]];
  display_edge[i][1] = projected_v[edges[i][1]];
  // determine if projected vertex is outside, 1 or 2 outside
  Projection_3d p_v1 = projected_v[edges[i][0]];
  Projection_3d p_v2 = projected_v[edges[i][1]];
  int v1_out = 0, v2_out = 0; //flag for determining outside
  int p_v1_Xmin_out = 0;
  int p_v1_Xmax_out = 0;
  int p_v1_Ymin_out = 0;
  int p_v1_Ymax_out = 0;
  int p_v2_Xmin_out = 0;
  int p_v2_Xmax_out = 0;
  int p_v2_Ymin_out = 0;
  int p_v2_Ymax_out = 0;
  if (p_v1.x < X_MIN) p_v1_Xmin_out = 1;
  if (p_v1.x > X_MAX) p_v1_Xmax_out = 1;
  if (p_v1.y < Y_MIN) p_v1_Ymin_out = 1;
  if (p_v1.y > Y_MAX) p_v1_Ymax_out = 1;
  if (p_v2.x < X_MIN) p_v2_Xmin_out = 1;
  if (p_v2.x > X_MAX) p_v2_Xmax_out = 1;
  if (p_v2.y < Y_MIN) p_v2_Ymin_out = 1;
  if (p_v2.y > Y_MAX) p_v2_Ymax_out = 1;
  // find which is outside
  if (p_v1_Xmin_out || p_v1_Xmax_out || p_v1_Ymin_out || p_v1_Ymax_out) {
    // v1 is outside
    v1_out = 1;
    // printf("%d is outside.\n", edges[i][0]);
    // compute_edge_intersection(display_edge[i][0], display_edge[i][1], in_view_edge, i);
    // printf("--v1 x:%lf y:%lf\n", p_v1.x, p_v1.y);
  }
  if(p_v2_Xmin_out || p_v2_Xmax_out || p_v2_Ymin_out || p_v2_Ymax_out) {
    // v2 is outside
    v2_out = 1;
    // printf("--v2 x:%lf y:%lf\n", p_v2.x, p_v2.y);
  }

  if (v1_out && v2_out) {
    // both outside
    // in_view_edge[i] = 0;
    printf("%d %d both outside(x,y).\n",edges[i][0], edges[i][1]);
    printf("x:%lf y:%lf , x:%lf y:%lf\n", p_v1.x, p_v1.y, p_v2.x, p_v2.y);
    // determine whether line has intersections on edges
    compute_edge_intersection(display_edge, in_view_edge, i);
  } else if (v1_out || v2_out) {
    // v1 or v2 outside
    printf("one vertex is outside(x,y).\n");
    printf("v1:%d x:%lf y:%lf\n", edges[i][0], p_v1.x, p_v1.y);
    printf("v2:%d x:%lf y:%lf\n", edges[i][1], p_v2.x, p_v2.y);
    // draw vertices
    if(v1_out) draw_vertices[edges[i][1]] = 1;
    if(v2_out) draw_vertices[edges[i][0]] = 1;
    // don't need to put outside vertex to matched position
    // function will change the outside vertex
    compute_edge_intersection(display_edge, in_view_edge, i);
  } else {
    // both inside
    printf("%d %d both inside(x,y).\n", edges[i][0], edges[i][1]);
    // display_edge[i][0] = projected_v[edges[i][0]];
    // display_edge[i][1] = projected_v[edges[i][1]];
    draw_vertices[edges[i][0]] = 1;
    draw_vertices[edges[i][1]] = 1;
    in_view_edge[i] = 1;
  }

}

// for Cube edge
void project_edge_point(Position_3d cube_vertices[],Projection_3d projected_v[], 
                        Projection_3d display_edge[][2], int edges[][2],
                        int in_view_edge[], int draw_vertices[], int num, int f, int d) {
  int i;
  // initialize lines which should be drawn
  for (i = 0; i < num; i++) {
    in_view_edge[i] = 0;
    draw_vertices[i] = 0;
  }
  for (i = 0; i < num; i++) {
    Position_3d v1 = cube_vertices[edges[i][0]];
    Position_3d v2 = cube_vertices[edges[i][1]];

    int v1_in_view = 0, v2_in_view = 0;
    if (v1.z > -d) v1_in_view = 1;
    if (v2.z > -d) v2_in_view = 1;

    // bool v1_in_view = (v1.z + d > 0);
    // bool v2_in_view = (v2.z + d > 0);
    // printf("v%d: X: %f Y: %f Z: %f\n", edges[i][0], v1.x, v1.y, v1.z);
    // printf("v%d: X: %f Y: %f Z: %f\n", edges[i][1], v2.x, v2.y, v2.z);
    // printf("in view, v1: %d, v2: %d\n", v1_in_view, v2_in_view);
    // printf("t1:%d t2:%d\n", v1_in_view, v2_in_view);

    // 若兩點都在視角內，直接投影
    // 要判斷是否在視角平面
    if (v1_in_view && v2_in_view) {
      projected_v[edges[i][0]] = project(v1, f, d);
      projected_v[edges[i][1]] = project(v2, f, d);
      // draw_line(proj_in, proj_inters); // 畫線函數
      printf("%d %d both inside(z).\n", edges[i][0], edges[i][1]);
      printf("v1.z:%f v2.z:%f both inside(z).\n", v1.z, v2.z);
      // maybe here made a bug
      determine_in_vision_edge(projected_v, display_edge, edges, in_view_edge, draw_vertices, i);
      // display_edge[i][0] = projected_v[edges[i][0]];
      // display_edge[i][1] = projected_v[edges[i][1]];
      // in_view_edge[i] = 1;
    }
    // 若只有一點在視角內，則計算交點，並畫到該交點為止
    else if (v1_in_view || v2_in_view) {
      Position_3d in_view = v1_in_view ? v1 : v2;
      Position_3d out_view = v1_in_view ? v2 : v1;

      // 計算交點
      double t = (-d - in_view.z) / (out_view.z - in_view.z);
      // 交點
      Position_3d intersection = {in_view.x + t * (out_view.x - in_view.x),
                                  in_view.y + t * (out_view.y - in_view.y), -d};
      // printf("inter: x:%f y:%f\n", intersection.x, intersection.y);
      if (v1_in_view) {
        printf("%d is outside(z).\n", edges[i][1]);
        // draw_vertices[edges[i][1]] = 0;
        projected_v[edges[i][0]] = project(in_view, f, d);
        projected_v[edges[i][1]].x = intersection.x * f;
        projected_v[edges[i][1]].y = intersection.y * f;
      } else {
        printf("%d is outside(z).\n", edges[i][0]);
        // draw_vertices[edges[i][0]] = 0;
        projected_v[edges[i][0]].x = intersection.x * f;
        projected_v[edges[i][0]].y = intersection.y * f;
        projected_v[edges[i][1]] = project(in_view, f, d);
      }
      printf("inter: x:%f y:%f\n", intersection.x, intersection.y);
      // draw_line(proj_in, proj_inters); // 畫到交點
      // in_view_edge[i] = 1;

      // determine if projected vertex is outside, 1 or 2 outside
      determine_in_vision_edge(projected_v, display_edge, edges, in_view_edge, draw_vertices, i);
    } else {
      // 若兩點都在視角外，則不顯示
      in_view_edge[i] = 0;
      // no projection
      printf("%d %d both outside(z).\n", edges[i][0], edges[i][1]);
      projected_zero(projected_v[edges[i][0]], projected_v[edges[i][1]]);
      projected_zero(display_edge[i][0], display_edge[i][1]);
    }
  }
}