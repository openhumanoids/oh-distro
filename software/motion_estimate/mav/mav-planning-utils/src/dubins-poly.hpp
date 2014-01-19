#ifndef __dubinspoly_hpp__
#define __dubinspoly_hpp__

#include "dubins.hpp"
#include "poly.hpp"
#include <lcmtypes/dubins_poly_segment_t.h>
#include <lcmtypes/dubins_poly_segment_list_t.h>

/*
 * abstract class that curves and lines inherit from
 *
 * all the functions that return paths, populate list<DubinsPathPrimitive *>
 * the type variable is {1,-1} if the path is an arc, and 0 if the path is a line
 */
class DubinsPolySegment {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DubinsPathPrimitive * dubins_primitive;
  DubinsArc * dubins_arc;
  DubinsLine * dubins_line;
  Polynomial poly;
  double theta_end;

  DubinsPolySegment(DubinsPathPrimitive * dubins_primitive);

  DubinsPolySegment(const dubins_poly_segment_t * msg);

  void to_dubins_poly_segment_t(dubins_poly_segment_t * msg);

  /**
   * transverse and axial derivatives at theta
   *
   * first row is transverse, second row is axial
   */
  void getTransAxialDerivatives(double theta, Eigen::MatrixXd & trans_axial_derivatives);
  void getStartTransAxialDerivatives(Eigen::MatrixXd & trans_axial_derivatives);
  void getEndTransAxialDerivatives(Eigen::MatrixXd & trans_axial_derivatives);

  /**
   * polynomial derivatives corresponding to the specified transverse derivatives
   */
  void getPolyDerivatives(const Eigen::VectorXd & trans_derivatives, Eigen::VectorXd & poly_derivatives);

  /**
   * arc length with respect to dtheta at the specified theta
   */
  double getArcLength(double theta);

  /**
   * axial unit vector along nominal dubins path
   */
  Eigen::Vector2d getAxialUnitVec(double theta);

  /**
   * transverse unit vector to left of nominal dubins path
   */
  Eigen::Vector2d getTransUnitVec(double theta);

  /**
   * unit vectors composed into rotation matrix
   */
  Eigen::Matrix2d getTransAxialRotationMatrix(double theta);

  /**
   * origin of trans axial frame in xy coordinates
   */
  Eigen::Vector2d getTransAxialOrigin(double theta);

  ~DubinsPolySegment();
};

typedef std::list<DubinsPolySegment *> DubinsPolyPathList;

std::ostream& operator<<(std::ostream& output, const DubinsPolySegment & seg);

/**
 * class acts as an iterator through the path and should contain all interface functions necessary to get path statistics for control, etc
 *
 * if it doesn't have it, add it!
 */
class DubinsPolyPath {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
  double theta; //parameter state for the current segment
  std::list<DubinsPolySegment *>::const_iterator it; //iterator for the current segment in the list
  const std::list<DubinsPolySegment *> * path; //path list
  std::list<DubinsPolySegment *> * path_nonconst; //path list
  DubinsPolySegment * cur_seg; //pointer to current segment, redundant with iterator, but useful

  bool path_owner;

public:
  bool loop_path;

  DubinsPolyPath(std::list<DubinsPolySegment *> & path, bool loop_path_arg = false, bool path_owner = false);

  DubinsPolyPath(const std::list<DubinsPolySegment *> & path, bool loop_path_arg = false);

  DubinsPolyPath(const dubins_poly_segment_list_t * msg);

  DubinsPolyPath(const DubinsPolyPath & other);

  ~DubinsPolyPath();

  dubins_poly_segment_list_t * to_dubins_poly_segment_list_t() const;

  void spliceAtBeginning(std::list<DubinsPolySegment *> & other_list);
  void spliceAtEnd(std::list<DubinsPolySegment *> & other_list);

  /**
   * step along the path by ds
   */
  bool step(double ds);

  /**
   * returns true if the path is empty
   */
  bool empty();

  /**
   * reset iterator to beginning
   */
  void reset();

  /**
   * sent iterator to end
   */
  void goToEnd();

  /**
   * current xy point
   */
  Eigen::Vector2d getPoint() const;

  /**
   * get heading along current point
   */
  double getHeading() const;

  /**
   * returns transverse offset from dubins path (can be used to discard bad polynomial optimizations)
   */
  double transOffset() const;

  /**
   * returns true if the current point is valid dubins-poly optimization (checks if we have negative R for arc segment)
   */
  bool isValidPoint() const;

  /**
   * 2xD matrix of derivatives, 0th column is the point, 1st column is 1st derivative, etc, D<=4
   */
  void getDerivatives(Eigen::MatrixXd & derivatives) const;

  /**
   * curvature (positive to left)
   */
  double getCurvature() const;

  /**
   * curvature and curvature derivative
   */
  void getCurvatureDerivatives(double * curvature, double * dcurvature) const;

  double getTheta();

  DubinsPolySegment * getCurSeg();

  /**
   * draw the path with resolution ds
   */
  void lcmgl_print(bot_lcmgl_t * lcmgl, double ds = .1, double z = 0.0) const;

  /**
   * draw the path with resolution ds up to the current point of the path iterator
   */
  void lcmgl_print_up_to_cur(bot_lcmgl_t * lcmgl, double ds = .1, double z = 0.0) const;

  /**
   * draw the underlying dubins path
   */
  void lcmgl_print_dubins(bot_lcmgl_t * lcmgl, double ds = .1, double z = 0.0) const;

  /**
   * print vectors at spacing ds corresponding to derivative der
   */
  void lcmgl_print_derivative_vec(bot_lcmgl_t * lcmgl, int der, double scale, double ds = .1) const;

  /**
   * print the trans axial coordinate frame along underlying dubins path (useful for debugging, but shouldn't be used otherwise
   */
  void lcmgl_print_trans_axial_coords(bot_lcmgl_t * lcmgl, double scale, double ds = .1) const;
};

double dubinsPolyGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, const Eigen::VectorXd & der_costs, const Eigen::VectorXd & start_trans_deriviatives,
    const Eigen::VectorXd & end_trans_deriviatives, int D, std::list<DubinsPolySegment *> & path);
double dubinsPolyGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, const Eigen::VectorXd & der_costs, int D, std::list<DubinsPolySegment *> & path);
double dubinsPolyGetPath(const std::list<DubinsPathPrimitive *> & dubins_path, const Eigen::VectorXd & der_costs,
    const Eigen::VectorXd & start_trans_derivatives, const Eigen::VectorXd & end_trans_derivatives, int D,
    std::list<DubinsPolySegment *> & path);

dubins_poly_segment_list_t * to_dubins_poly_segment_list(const std::list<DubinsPolySegment *> & path);
void toDubinsPolyPath(const dubins_poly_segment_list_t * msg, std::list<DubinsPolySegment *> & path);

//------------- Utility Functions for DubinsPoly Paths ---------------------
double dubinsPolyGetPathLength(const std::list<DubinsPolySegment *> & path);

void dubinsPolyFreePath(std::list<DubinsPolySegment *> & path);

void dubinsPoly_lcmgl_printPath(const std::list<DubinsPolySegment *> & path, bot_lcmgl_t * lcmgl);

#endif
