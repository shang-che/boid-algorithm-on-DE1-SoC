#include <math.h>
#include <stdio.h>
// for keyboard
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#define M_PI 3.14159265358979323846
// f (focal length)是視場深度(焦距)
// d 是攝影機到投影平面的距離
// 我們假設 f = 1 和 d=3。
// #define f 1
// #define d 3

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
  double rx = v.x;
  double ry = v.y;
  double rz = v.z;
  double x_2D = (rx * f) / (rz + d);
  double y_2D = (ry * f) / (rz + d);
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
