#include <iostream>
#include <bot_core/bot_core.h>
#include "../dubins.hpp"

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

int main(int argc, char ** argv)
{

  fprintf(stderr,
      "usage %s: any argument switches to floating heading mode, no argument corresponds to specified heading\n",
      argv[0]);

  bool theta_end_specified = true;
  if (argc > 1) {
    theta_end_specified = false;
  }

  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_t * lcmgl = bot_lcmgl_init(lcm, "dubins tester");

  list<DubinsPathPrimitive *> path = list<DubinsPathPrimitive *>();

  double R = 1.3;
  double box_size = 5;
  int num_iterations = 100;
  double sleep_time = .5;

  Vector3d start_vec, end_vec;
  Vector2d xy_start, xy_end;
  double theta_start, theta_end, path_length;

  int it = 0;

  while (it < num_iterations) {
    start_vec = Vector3d::Random();
    end_vec = Vector3d::Random();

    start_vec(2) *= M_PI;
    end_vec(2) *= M_PI;

    xy_start = start_vec.block(0, 0, 2, 1) *= box_size;
    xy_end = end_vec.block(0, 0, 2, 1) *= box_size;
    theta_start = start_vec(2);
    theta_end = end_vec(2);

    if (theta_end_specified) {
      path_length = dubinsGetPath(R, xy_start, theta_start, xy_end, theta_end, path);
    }
    else {
      path_length = dubinsGetPath(R, xy_start, theta_start, xy_end, path);
      path_length = dubinsGetDistance(R, xy_start, theta_start, xy_end);
      if (path_length<INFINITY)      {
        theta_end = (*path.rbegin())->getEndPose()[2];
      }
    }
  cout << path_length << endl;

  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_color3f(lcmgl, 0, 0, 1);

  dubins_lcmgl_printPath(path, lcmgl);

  Vector2d xy_start_vec_draw = xy_start + R * angleToVec(theta_start);
  Vector2d xy_end_vec_draw = xy_end + R * angleToVec(theta_end);
  bot_lcmgl_color3f(lcmgl, 0, 1, 0);
  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex2d(lcmgl, xy_start(0), xy_start(1));
  bot_lcmgl_vertex2d(lcmgl, xy_start_vec_draw(0), xy_start_vec_draw(1));

  bot_lcmgl_color3f(lcmgl, 1, 0, 0);
  bot_lcmgl_vertex2d(lcmgl, xy_end(0), xy_end(1));
  bot_lcmgl_vertex2d(lcmgl, xy_end_vec_draw(0), xy_end_vec_draw(1));
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_switch_buffer(lcmgl);

  list<DubinsPathPrimitive *>::iterator path_it;
  for (path_it = path.begin(); path_it != path.end(); path_it++) {
    DubinsPathPrimitive * prim_path = *path_it;
    int type = prim_path->type;
    if (type == 0) {
      DubinsLine * line_path = (DubinsLine *) (*path_it);
      cout << *line_path << endl;
    }
    else {
      DubinsArc * arc_path = (DubinsArc *) (*path_it);
      cout << *arc_path << endl;
    }
  }

  cout << endl << endl;

  dubinsFreePath(path);
  usleep(sleep_time * 1e6);
  it++;

}
}

