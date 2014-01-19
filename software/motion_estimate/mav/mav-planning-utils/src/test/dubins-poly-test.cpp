#include <iostream>
#include <bot_core/bot_core.h>
#include "../dubins-poly.hpp"

using namespace std;
using namespace Eigen;
using namespace eigen_utils;

int main(int argc, char ** argv)
{

  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_t * lcmgl = bot_lcmgl_init(lcm, "dubins poly tester");

  list<DubinsPolySegment *> path_list = list<DubinsPolySegment *>();

  int path_case = 1;
  if (argc > 1) {
    path_case = atoi(argv[1]);
  }
  else
  {
    fprintf(stderr, "usage: %s <path_case>, default 0\n", argv[0], path_case);
    path_case = 1;
  }
//
//  double R = 8.0;
//  double V = 1.0;
//
//  int N = 8;
//  int D = 4;
//
//  VectorXd start_trans_derivatives(D), end_trans_derivatives(D), der_costs(N + 1);
//  start_trans_derivatives.setZero();
//  end_trans_derivatives.setZero();
//  der_costs.setZero();
//
//  double accel_cost = .01;
//  int der_cost = 4;
//  der_costs(der_cost) = accel_cost * pow(R, der_cost);
//  der_costs(0) = 1;
//
//  Vector2d xy_start, xy_end;
//  double theta_start, theta_end;
//
//  switch (path_case) {
//  case 1:
//    xy_start << 0, 0;
//    xy_end << -R, 3 * R;
//    theta_start = 0;
//    theta_end = M_PI / 2.0;
//    break;
//  case 2:
//    xy_start << 0, 0;
//    xy_end << -R, 2 * R;
//    theta_start = 0;
//    theta_end = -M_PI;
//    break;
//  case 3:
//    xy_start << 0, 0;
//    xy_end << 0, 4 * R;
//    theta_start = 0;
//    theta_end = 0;
//    break;
//  default:
//    xy_start << R, 0;
//    xy_end << -3 * R, 2 * R;
//    theta_start = M_PI / 2.0;
//    theta_end = M_PI / 2.0;
//    break;
//  }
//
//  double poly_cost = dubinsPolyGetPath(R, xy_start, theta_start, xy_end, theta_end, der_costs, D, path_list);

  double R = 7.0;
  double V = 8.0;
  double z = 10.0;
  bool loop_path = true;

  int N = 8;
  int D = 4;

  VectorXd start_trans_derivatives(D), end_trans_derivatives(D), der_costs(N + 1);
  start_trans_derivatives.setZero();
  end_trans_derivatives.setZero();
  der_costs.setZero();

  double accel_cost = .01;
  int der_cost = 4;
  der_costs(0) = 1;
  der_costs(der_cost) = accel_cost * pow(R, der_cost);


  //  der_costs(0) = 1;
//  der_costs(1) = 1;
////  der_costs(2) = .01;
//  der_costs(3) = .1;
//  der_costs(4) = .0001;

  Vector2d xy_start, xy_wpt1;
  double theta_start, theta_wpt1;

  xy_start << 0, 0;
  theta_start = 0;

  xy_wpt1 << -R, 3 * R;
  theta_wpt1 = M_PI / 2.0;
  dubinsPolyGetPath(R, xy_start, theta_start, xy_start, theta_start, der_costs, D, path_list);
//  dubinsPolyGetPath(R, xy_wpt1, theta_wpt1, xy_start, theta_start, der_costs, D, path_list);

  DubinsPolyPath path(path_list);

  bot_lcmgl_line_width(lcmgl, 3);
  bot_lcmgl_color3f(lcmgl, 0, 0, 1);
  path.lcmgl_print(lcmgl);

//  bot_lcmgl_color3f(lcmgl, 1, 0, 0);
//  path.lcmgl_print_derivative_vec(lcmgl, 1, .1 * R, .2 * R);
//
//  bot_lcmgl_color3f(lcmgl, 0, 1, 0);
//  path.lcmgl_print_derivative_vec(lcmgl, 2, .2 * R * R, .01 * R);

//  bot_lcmgl_color3f(lcmgl, 1, 1, 0);
//  path.lcmgl_print_derivative_vec(lcmgl, 3, .5 * R * R * R, .2 * R);
//
//  bot_lcmgl_color3f(lcmgl, 0, 0, 0);
//  path.lcmgl_print_trans_axial_coords(lcmgl, .5 * R);

  {
    double ds = .1;
    path.reset();
    bool ended = false;
    int num_steps = 0;
    while (!ended) {
      ended = path.step(ds);
      num_steps++;
    }

    MatrixXd xyc(2, num_steps), xydc(2, num_steps), xyddc(2, num_steps), xydddc(2, num_steps);
    MatrixXd xy_der(2, 4);
    ended = false;
    int ii = 0;
    path.reset();
    while (!ended) {
      path.getDerivatives(xy_der);
//      eigen_dump(xy_der);
      xyc.col(ii) = xy_der.col(0);
      xydc.col(ii) = xy_der.col(1);
      xyddc.col(ii) = xy_der.col(2);
      xydddc.col(ii) = xy_der.col(3);
      ended = path.step(ds);
      ii++;
    }

//    eigen_matlab_dump(xyc);
//    eigen_matlab_dump(xydc);
//    eigen_matlab_dump(xyddc);
//    eigen_matlab_dump(xydddc);

    MatrixXd xy_int(2, num_steps), xyd_int(2, num_steps), xydd_int(2, num_steps);
    xy_int = xyc;
    xyd_int = xydc;
    xydd_int = xyddc;
    for (ii = 1; ii < num_steps; ii++) {
      xydd_int.col(ii) = xydd_int.col(ii - 1) + ds * xydddc.col(ii);
      xyd_int.col(ii) = xyd_int.col(ii - 1) + ds * xydd_int.col(ii);
      xy_int.col(ii) = xy_int.col(ii - 1) + ds * xyd_int.col(ii);
    }

//    eigen_dump(xyddd);

    bot_lcmgl_line_width(lcmgl, 3);
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);

    for (ii = 1; ii < num_steps; ii++) {
      lcmglVertex3d(xy_int(0,ii), xy_int(1,ii), 1);
    }
    bot_lcmgl_end(lcmgl);

  }

  bot_lcmgl_switch_buffer(lcmgl);

  list<DubinsPolySegment *>::iterator path_it;
  for (path_it = path_list.begin(); path_it != path_list.end(); path_it++) {
    DubinsPolySegment * prim_path = *path_it;
    VectorXd coeffs = prim_path->poly.coeffs;
    eigen_matlab_dump(coeffs);
  }

//  double box_size = 5;
//  double sleep_time = 1.0;
//
//  while (true) {
//    start_vec = Vector3d::Random();
//    end_vec = Vector3d::Random();
//
//    start_vec(2) *= M_PI;
//    end_vec(2) *= M_PI;
//
//    xy_start = start_vec.topRows(2) *= box_size;
//    xy_end = end_vec.topRows(2) *= box_size;
//    theta_start = start_vec(2);
//    theta_end = end_vec(2);
//
//    eigen_matlab_dump(R);
//    eigen_matlab_dump(xy_start);
//    eigen_matlab_dump(theta_start);
//    eigen_matlab_dump(xy_end);
//    eigen_matlab_dump(theta_end);
//    eigen_matlab_dump(der_costs);
//    eigen_matlab_dump(start_trans_derivatives);
//    eigen_matlab_dump(end_trans_derivatives);
//    eigen_matlab_dump(D);
//
//    poly_cost = dubinsPolyGetPath(R, xy_start, theta_start, xy_end, theta_end, der_costs, start_trans_derivatives,
//        end_trans_derivatives, D, path);
//
//    cout << poly_cost << endl;
//
//    bot_lcmgl_line_width(lcmgl, 3);
//    bot_lcmgl_color3f(lcmgl, 0, 0, 1);
//
//    dubinsPoly_lcmgl_printPath(path, lcmgl);
//
//    Vector2d xy_start_vec_draw = xy_start + R * angleToVec(theta_start);
//    Vector2d xy_end_vec_draw = xy_end + R * angleToVec(theta_end);
//    bot_lcmgl_color3f(lcmgl, 0, 1, 0);
//    bot_lcmgl_begin(lcmgl, GL_LINES);
//    bot_lcmgl_vertex2d(lcmgl, xy_start(0), xy_start(1));
//    bot_lcmgl_vertex2d(lcmgl, xy_start_vec_draw(0), xy_start_vec_draw(1));
//
//    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
//    bot_lcmgl_vertex2d(lcmgl, xy_end(0), xy_end(1));
//    bot_lcmgl_vertex2d(lcmgl, xy_end_vec_draw(0), xy_end_vec_draw(1));
//    bot_lcmgl_end(lcmgl);
//
//    bot_lcmgl_switch_buffer(lcmgl);
//
//    list<DubinsPolySegment *>::iterator path_it;
//    for (path_it = path.begin(); path_it != path.end(); path_it++) {
//      DubinsPolySegment * prim_path = *path_it;
//      int type = prim_path->type;
//      if (type == 0) {
//        DubinsLine * line_path = (DubinsLine *) (*path_it);
//        cout << *line_path << endl;
//      }
//      else {
//        DubinsArc * arc_path = (DubinsArc *) (*path_it);
//        cout << *arc_path << endl;
//      }
//    }
//
//    cout << endl << endl;
//
//  dubinsPolyFreePath(path);
//  usleep(sleep_time * 1e6);
//
//}
}

