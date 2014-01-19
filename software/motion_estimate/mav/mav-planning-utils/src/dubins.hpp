#ifndef __dubins_hpp__
#define __dubins_hpp__

#include <Eigen/Dense>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_vis/gl_util.h>
#include <bot_core/math_util.h>
#include <list>
#include <assert.h>
#include <eigen_utils/eigen_utils.hpp>
#include <iostream>

#include <lcmtypes/dubins_primitive_t.h>
#include <lcmtypes/dubins_primitive_list_t.h>
#include <lcmtypes/dubins/primitive_list_t.hpp>

#define ARC_DRAW_RESOLUTION 180.0 //number of steps per pi
/*
 * abstract class that curves and lines inherit from
 *
 * all the functions that return paths, populate list<DubinsPathPrimitive *>
 * the type variable is {1,-1} if the path is an arc, and 0 if the path is a line
 */
class DubinsPathPrimitive {
public:
  virtual ~DubinsPathPrimitive()
  {
  }

  int type;
  double curvature;
  virtual void lcmgl_print(bot_lcmgl_t * lcmgl) const = 0;
  virtual double getLength() const = 0;
  virtual Eigen::Vector3d getStartPose()const = 0;
  virtual Eigen::Vector3d getEndPose() const = 0;

  //copy method
  DubinsPathPrimitive * copy() const;

  //creates and returns msg, must be destroyed externally
  virtual void to_dubins_primitive_t(dubins_primitive_t * msg) const = 0;
  virtual void to_dubins_primitive_t(dubins::primitive_t * msg) const = 0;
  static DubinsPathPrimitive * from_dubins_primitive_t(const dubins_primitive_t * msg);
  static DubinsPathPrimitive * from_dubins_primitive_t(const dubins::primitive_t * msg);
};

typedef std::list<DubinsPathPrimitive *> DubinsPath;

/*
 * populates path with the optimal dubins path between [xy_start, theta_start]->[xy_end, theta_end]
 *
 * returns the path length
 */
double dubinsGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, DubinsPath & path);
double dubinsGetPath(double R, const Eigen::Vector3d & x_start, const Eigen::Vector3d & x_end,
    DubinsPath & path);

/**
 * L: +1 is ccw, -1 is cw, 0 is straight
 */
bool dubinsGetPathWord(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, DubinsPath & path, const int L[3]);

/**
 * functions for unspecified final heading
 */
double dubinsGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    DubinsPath & path);
double dubinsGetPath(double R, const Eigen::Vector3d & xytheta_start, const Eigen::Vector2d & xy_end,
    DubinsPath & path);

/**
 * step along a dubins path by distance step_dl
 */
double dubinsAdvanceCarrot(const DubinsPath & path_data,
    DubinsPath::const_iterator & path_it, Eigen::Vector3d & x_carrot, int * step_ccw,
    double step_dl);

/*
 * gets the path length corresponding to the optimal word, and places the word in the optional argument
 */
double dubinsGetDistance(double R, const Eigen::Vector2d & xy_start, double theta_start,
    const Eigen::Vector2d & xy_end, double theta_end, int word[3] = NULL);
double dubinsGetDistance(double R, const Eigen::Vector3d & x_start, const Eigen::Vector3d & x_end, int word[3] = NULL);

/**
 * same for unspecified heading
 */
double dubinsGetDistance(double R, const Eigen::Vector2d & xy_start, double theta_start,
    const Eigen::Vector2d & xy_end);
double dubinsGetDistance(double R, const Eigen::Vector3d & xytheta_start, const Eigen::Vector2d & xy_end);

//------------- Utility Functions for Dubins Paths ---------------------
double dubinsGetPathLength(const DubinsPath & path);

Eigen::ArrayXXd dubinsPathRasterize(const DubinsPath & path, double step_size);

void dubinsFreePath(DubinsPath & path);

void dubins_lcmgl_printPath(const DubinsPath & path, bot_lcmgl_t * lcmgl);

dubins_primitive_list_t * to_dubins_primitive_list_t(const DubinsPath & path);
void toDubinsPath(const dubins_primitive_list_t * msg, DubinsPath & path);
void toDubinsPath(const dubins::primitive_list_t * msg, DubinsPath & path);

//-------------- Path Element Classes: Line, Arc -----------------------
class DubinsLine: public DubinsPathPrimitive {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d start_pt;
  Eigen::Vector2d end_pt;
  Eigen::Vector2d unit_vec; //unit vector along line
  Eigen::Vector2d vec; //vector from start to end
  Eigen::Vector2d left_unit_vec; //unit vector perpendicular to the left of the line segment
  double length; //line length
  double theta; //line heading

  DubinsLine(const Eigen::Vector2d & start_pt, const Eigen::Vector2d & end_pt);
  DubinsLine(const Eigen::Vector2d & start_pt, const Eigen::Vector2d & end_pt, double theta);
  DubinsLine(const dubins_primitive_t * msg);
  DubinsLine(const dubins::primitive_t * msg);

  void lcmgl_print(bot_lcmgl_t * lcmgl) const;

  double getLength() const;

  Eigen::Vector3d getStartPose() const;
  Eigen::Vector3d getEndPose() const;

  void to_dubins_primitive_t(dubins_primitive_t * msg) const;
  void to_dubins_primitive_t(dubins::primitive_t * msg) const;

  ~DubinsLine()
  {

  }
};

std::ostream& operator<<(std::ostream& output, const DubinsLine & line);

class DubinsArc: public DubinsPathPrimitive {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d center;
  double R;
  int ccw;
  double start_angle;
  double end_angle;
  double sweep_angle;
  double length;

  DubinsArc(const Eigen::Vector2d & center, double R, double start_angle, double end_angle, int ccw);

  DubinsArc(const dubins_primitive_t * msg);
  DubinsArc(const dubins::primitive_t * msg);

  double getLength() const;

  Eigen::Vector2d getEndPoint() const;

  Eigen::Vector2d getStartPoint() const;

  Eigen::Vector3d getStartPose() const;
  Eigen::Vector3d getEndPose() const;

  void lcmgl_print(bot_lcmgl_t * lcmgl) const;

  void to_dubins_primitive_t(dubins_primitive_t * msg) const;
  void to_dubins_primitive_t(dubins::primitive_t * msg) const;

  ~DubinsArc()
  {

  }

};

std::ostream& operator<<(std::ostream& output, const DubinsArc & arc);

#endif
