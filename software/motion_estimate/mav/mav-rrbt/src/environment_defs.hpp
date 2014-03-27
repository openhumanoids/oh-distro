#ifndef _ENVIRONMENT_DEFS_HPP
#define _ENVIRONMENT_DEFS_HPP

#include "drawing_defs.hpp"
#include <iostream>

#define X_TYPEDEFS typedef Eigen::Matrix<double, N, 1> X; \
                   typedef Eigen::Matrix<double, N, N> StateCov;

template<int N>
class CSpaceInterface {
public:
  X_TYPEDEFS

  virtual bool collides(const X & x) = 0;

  virtual void sample(X & x_sample) = 0;

  virtual bool collides_cc(const X & x, const StateCov & cov, double rho) = 0;

  virtual void lcmgl_print(bot_lcmgl_t * lcmgl)
  {

  }

};

template<int N>
class Region {
public:
  X_TYPEDEFS

  virtual bool contains(const X & x) = 0;

  virtual void lcmgl_print(bot_lcmgl_t * lcmgl) = 0;

  virtual bool collides_cc(const X & x, const StateCov & sigma, double rho) = 0;

  virtual bool contains_cc(const X & x, const StateCov & sigma, double rho) = 0;

  virtual void sample(X & s_samp) = 0;

  virtual ~Region()
  {

  }
};

template<int N>
class HyperBox: public Region<N> {
public:
  X_TYPEDEFS

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  X minCorner;
  X maxCorner;

  X center;
  X size;

  HyperBox(const X & center, const X & size)
  {
    this->center = center;
    this->size = size;

    int ii;
    for (ii = 0; ii < N; ii++) {
      minCorner(ii) = center(ii) - size(ii) / 2;
      maxCorner(ii) = center(ii) + size(ii) / 2;
    }
  }

  HyperBox()
  {
    this->center = X::Constant(0);
    this->size = X::Constant(10);

    int ii;
    for (ii = 0; ii < N; ii++) {
      minCorner(ii) = center(ii) - size(ii) / 2;
      maxCorner(ii) = center(ii) + size(ii) / 2;
    }
  }

  ~HyperBox()
  {

  }

  bool contains(const X & x)
  {
    int ii;
    for (ii = 0; ii < N; ii++) {
      if ((x(ii) < minCorner(ii)) || (x(ii) > maxCorner(ii))) {
        return false;
      }
    }
    return true;
  }

  bool contains_cc(const X & x, const StateCov & sigma_total, double rho)
  {

    int ii;
    for (ii = 0; ii < N; ii++) {
      if (x(ii) < (minCorner(ii) + rho * sqrt(sigma_total(ii, ii)))
          || x(ii) > (maxCorner(ii) - rho * sqrt(sigma_total(ii, ii)))) {
        return false;
      }
    }
    return true;
  }

  bool collides_cc(const X & x, const StateCov & sigma_total, double rho)
  {

    int ii;
    for (ii = 0; ii < N; ii++) {
      if (x(ii) < (minCorner(ii) - rho * sqrt(sigma_total(ii, ii)))
          || x(ii) > (maxCorner(ii) + rho * sqrt(sigma_total(ii, ii)))) {
        return false;
      }
    }
    return true;
  }

  void sample(X & x_samp)
  {
    x_samp = X::Random();
    x_samp.array() = x_samp.array() * this->size.array() / 2;
    x_samp += this->center;
  }

  virtual void lcmgl_print(bot_lcmgl_t * lcmgl)
  {
    if (N == 3) {
      bot_lcmgl_color3f(lcmgl, 0, 0, 0);
      bot_lcmgl_line_width(lcmgl, 4);
      lcmgl_print_3d_frame(lcmgl);
    }
    else
      lcmgl_print_z(lcmgl, Z_REGIONS);
  }

  virtual void lcmgl_print_z(bot_lcmgl_t * lcmgl, double z)
  {
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->minCorner(1), z);
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->minCorner(1), z);
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->maxCorner(1), z);
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->maxCorner(1), z);
  }

  virtual void lcmgl_print_3d(bot_lcmgl_t * lcmgl)
  {
    double xyz[3];
    float size[3];

    xyz[0] = this->center(0);
    xyz[1] = this->center(1);
    xyz[2] = this->center(2);
    size[0] = this->size(0);
    size[1] = this->size(1);
    size[2] = this->size(2);

    bot_lcmgl_box(lcmgl, xyz, size);
  }

  virtual void lcmgl_print_3d_frame(bot_lcmgl_t * lcmgl)
  {
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->minCorner(1), this->minCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->minCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->maxCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->maxCorner(1), this->minCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->minCorner(1), this->minCorner(2));
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->minCorner(1), this->minCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->minCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->maxCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->maxCorner(1), this->minCorner(2));
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->minCorner(1), this->minCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->minCorner(1), this->minCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->maxCorner(1), this->minCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->maxCorner(1), this->minCorner(2));
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->minCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->minCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->maxCorner(0), this->maxCorner(1), this->maxCorner(2));
    bot_lcmgl_vertex3d(lcmgl, this->minCorner(0), this->maxCorner(1), this->maxCorner(2));
    bot_lcmgl_end(lcmgl);
  }
};

typedef HyperBox<2> Box;

template<int N>
class HyperSphere: public Region<N> {
public:
  X_TYPEDEFS

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  X center;
  double R;
  HyperSphere(const X & center, double R)
  {
    this->center = center;
    this->R = R;
  }

  HyperSphere()
  {
    this->center = X::Zero();
    this->R = 1;
  }

  ~HyperSphere()
  {

  }

  bool contains(const X & x)
  {
    X diff = this->center - x;
    double dist = diff.norm();
    if (dist < this->R)
      return true;
    else
      return false;
  }

  bool contains_cc(const X & x, const StateCov & sigma_total, double rho)
  {
    X diff = this->center - x;
    StateCov sigma_llt = sigma_total.llt().matrixL();
    StateCov lower_triangle = this->R * StateCov::Identity() - rho * sigma_llt;

    for (int ii = 0; ii < N; ii++) { //if the lower triangular matrix is not positive on the diagonal, the corresponding ellipse is degenerate, and thus cannot contain a point
      if (lower_triangle(ii, ii) < 0.0)
        return false;
    }

    StateCov ellipsoid_mat = lower_triangle * lower_triangle.transpose();

    double weighted_dist = diff.transpose() * ellipsoid_mat.llt().solve(diff);
    bool collision = weighted_dist < 1.0;
    return collision;
  }

  bool collides_cc(const X & x, const StateCov & sigma_total, double rho)
  {
    X diff = this->center - x;
    StateCov sigma_llt = sigma_total.llt().matrixL();
    StateCov lower_triangle = rho * sigma_llt + this->R * StateCov::Identity();
    StateCov ellipsoid_mat = lower_triangle * lower_triangle.transpose();
    double weighted_dist = diff.transpose() * ellipsoid_mat.llt().solve(diff);
    bool collision = weighted_dist < 1.0;
    return collision;
  }

  void sample(X & x_samp)
  {
    do {
      x_samp = X::Random();
      x_samp.array() = x_samp.array() * this->R;
      x_samp += this->center;
    } while (!this->contains(x_samp));
  }

  void lcmgl_print(bot_lcmgl_t * lcmgl)
  {
    double xyz[] = { this->center(0), this->center(1), Z_REGIONS_CIRCLE };
    bot_lcmgl_circle(lcmgl, xyz, this->R);
  }
};

typedef HyperSphere<2> Circle;

template<int N>
class RegionCSpace: public CSpaceInterface<N> {
public:
  X_TYPEDEFS
  typename std::list<Region<N> *> obstacles;
  Region<N> * bounding_region;

  RegionCSpace(Region<N> * bounding_region)
  {
    this->bounding_region = bounding_region;
  }

  void addObstacle(Region<N> * obstacle)
  {
    this->obstacles.push_front(obstacle);
  }

  virtual bool collides(const X & x)
  {
    typename std::list<Region<N> *>::iterator obs_it;
    Region<N> * cur_obs;
    for (obs_it = this->obstacles.begin(); obs_it != this->obstacles.end(); obs_it++) {
      cur_obs = *obs_it;
      if (cur_obs->contains(x))
        return true;
    }

    if (!this->bounding_region->contains(x))
      return true;

    return false;
  }

  virtual void sample(X & x_samp)
  {
    do {
      this->bounding_region->sample(x_samp);
    } while (this->collides(x_samp));
  }

  virtual bool collides_cc(const X & x, const StateCov & cov, double rho)
  {
    typename std::list<Region<N> *>::iterator obs_it;
    Region<N> * cur_obs;
    for (obs_it = this->obstacles.begin(); obs_it != this->obstacles.end(); obs_it++) {
      cur_obs = *obs_it;
      if (cur_obs->collides_cc(x, cov, rho))
        return true;
    }

    if (!this->bounding_region->contains_cc(x, cov, rho))
      return true;

    return false;
  }

  virtual void lcmgl_print(bot_lcmgl_t * lcmgl)
  {
    if (N == 3) {
      typename std::list<Region<N> *>::iterator obs_it;
      Region<N> * cur_obs;
      for (obs_it = this->obstacles.begin(); obs_it != this->obstacles.end(); obs_it++) {
        cur_obs = *obs_it;
        cur_obs->lcmgl_print(lcmgl);
      }

    }
    else {
      typename std::list<Region<N> *>::iterator obs_it;
      Region<N> * cur_obs;
      bot_lcmgl_color3f(lcmgl, BT_OBSTACLE_COLOR);
      for (obs_it = this->obstacles.begin(); obs_it != this->obstacles.end(); obs_it++) {
        cur_obs = *obs_it;
        bot_lcmgl_begin(lcmgl, GL_POLYGON);
        cur_obs->lcmgl_print(lcmgl);
        bot_lcmgl_end(lcmgl);
      }

      bot_lcmgl_line_width(lcmgl, BT_BB_LINE_WIDTH);
      bot_lcmgl_begin(lcmgl, GL_LINE_LOOP);
      this->bounding_region->lcmgl_print(lcmgl);
      bot_lcmgl_end(lcmgl);
    }
  }

};

#endif

