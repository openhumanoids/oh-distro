#include "dubins.hpp"

using namespace Eigen;
/*
 * dubinsGets the path length corresponding to the desired word
 */
double dubinsGetDistanceWord(double R, const Eigen::Vector2d & xy_start, double theta_start,
    const Eigen::Vector2d & xy_end, double theta_end, const int word[3]);

/*
 * populates path with the dubins path corresponding to word.  Word must be one of: {{1,-1,1},{-1,1,-1},{1,0,1},{-1,0,-1},{-1,0,1},{1,0,-1}}
 *
 * returns true if the path exists
 */
bool dubinsGetPathWord(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, DubinsPath & path, const int word[3]);

using namespace eigen_utils;

double wrapTo2pi(double v)
{
  double result = bot_mod2pi(v);
  if (result < 0)
    return result + 2 * M_PI;
  else
    return result;
}

template<typename T>
DubinsPathPrimitive *_from_dubins_primitive(const T * msg)
{
  if (msg->type == 0) {
    return new DubinsLine(msg);
  }
  else {
    return new DubinsArc(msg);
  }
}
DubinsPathPrimitive * DubinsPathPrimitive::from_dubins_primitive_t(const dubins_primitive_t * msg)
{
  return _from_dubins_primitive(msg);
}
DubinsPathPrimitive * DubinsPathPrimitive::from_dubins_primitive_t(const dubins::primitive_t * msg)
{
  return _from_dubins_primitive(msg);
}

DubinsLine::DubinsLine(const Eigen::Vector2d & start_pt, const Eigen::Vector2d & end_pt)
{
  this->start_pt = start_pt;
  this->end_pt = end_pt;
  this->vec = end_pt - start_pt;
  this->length = this->vec.norm();
  if (this->length < 1e-11) {
    printf("warning: creating zero length dubins line using 2 argument constructor\n");
  }
  this->theta = atan2Vec(this->vec);
  this->unit_vec = this->vec / this->length;
  this->type = 0;
  this->curvature = 0;
  this->left_unit_vec << -this->unit_vec(1), this->unit_vec(0);
}

DubinsLine::DubinsLine(const Eigen::Vector2d & start_pt, const Eigen::Vector2d & end_pt, double theta)
{
  this->start_pt = start_pt;
  this->end_pt = end_pt;
  this->vec = end_pt - start_pt;
  this->length = this->vec.norm();
  this->theta = theta;
  this->unit_vec = angleToVec(this->theta);
  this->type = 0;
  this->curvature = 0;
  this->left_unit_vec << -this->unit_vec(1), this->unit_vec(0);
}

DubinsLine::DubinsLine(const dubins_primitive_t * msg) :
    start_pt(Map<const Vector2d>(msg->start_pt)), end_pt(Map<const Vector2d>(msg->end_pt)),
        theta(msg->theta)
{
  this->type = msg->type;
  if (msg->type != 0) {
    fprintf(stderr, "ERROR: constructing dubins line from lcm message dubins_primitive_t of wrong type %d\n",
        msg->type);
  }
  this->vec = end_pt - start_pt;
  this->length = this->vec.norm();
  this->unit_vec = angleToVec(this->theta);
  this->curvature = 0;
  this->left_unit_vec << -this->unit_vec(1), this->unit_vec(0);
}

DubinsLine::DubinsLine(const dubins::primitive_t * msg) :
    start_pt(Map<const Vector2d>(msg->start_pt)), end_pt(Map<const Vector2d>(msg->end_pt)),
        theta(msg->theta)
{
  this->type = msg->type;
  if (msg->type != 0) {
    fprintf(stderr, "ERROR: constructing dubins line from lcm message dubins_primitive_t of wrong type %d\n",
        msg->type);
  }
  this->vec = end_pt - start_pt;
  this->length = this->vec.norm();
  this->unit_vec = angleToVec(this->theta);
  this->curvature = 0;
  this->left_unit_vec << -this->unit_vec(1), this->unit_vec(0);
}

void DubinsLine::lcmgl_print(bot_lcmgl_t * lcmgl) const
    {
  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex3d(lcmgl, this->start_pt(0), this->start_pt(1), 0);
  bot_lcmgl_vertex3d(lcmgl, this->end_pt(0), this->end_pt(1), 0);
  bot_lcmgl_end(lcmgl);
}

double DubinsLine::getLength() const
{
  return this->length;
}

Eigen::Vector3d DubinsLine::getStartPose() const
{
  Eigen::Vector3d pose;
  pose.block<2, 1>(0, 0) = this->start_pt;
  pose(2) = this->theta;
  return pose;
}

Eigen::Vector3d DubinsLine::getEndPose() const
{
  Eigen::Vector3d pose;
  pose.block<2, 1>(0, 0) = this->end_pt;
  pose(2) = this->theta;
  return pose;
}

template<typename T>
void _to_dubins_primitive_t(const DubinsLine * self, T * msg)
{
  msg->type = self->type;
  msg->theta = self->theta;
  Eigen::Map<Vector2d>(msg->start_pt) = self->start_pt;
  Eigen::Map<Vector2d>(msg->end_pt) = self->end_pt;
}
void DubinsLine::to_dubins_primitive_t(dubins_primitive_t * msg) const
    {
  _to_dubins_primitive_t(this, msg);
}
void DubinsLine::to_dubins_primitive_t(dubins::primitive_t * msg) const
    {
  _to_dubins_primitive_t(this, msg);
}

std::ostream& operator<<(std::ostream& output, const DubinsLine & line)
{
  output << "Line, start\n" << line.start_pt << "\nend:\n" << line.end_pt << std::endl;
  return output;
}

DubinsArc::DubinsArc(const Eigen::Vector2d & center, double R, double start_angle, double end_angle, int ccw)
{
  this->center = center;
  this->R = R;
  this->start_angle = bot_mod2pi(start_angle);
  this->end_angle = bot_mod2pi(end_angle);
  assert((ccw == 1 || ccw == -1));
  this->ccw = ccw;
  this->sweep_angle = wrapTo2pi(this->ccw * (this->end_angle - this->start_angle));
  this->length = this->sweep_angle * this->R;

  this->type = this->ccw;
  this->curvature = this->ccw * (1.0 / this->R);
}

DubinsArc::DubinsArc(const dubins_primitive_t * msg) :
    center(Map<const Vector2d>(msg->center)), R(msg->R), start_angle(bot_mod2pi(msg->start_angle)), end_angle(
        bot_mod2pi(msg->end_angle)), ccw(msg->ccw)
{
  this->sweep_angle = wrapTo2pi(this->ccw * (this->end_angle - this->start_angle));
  this->length = this->sweep_angle * this->R;
  this->type = this->ccw;
  this->curvature = this->ccw * (1.0 / this->R);

  if (msg->type != 1 && msg->type != -1) {
    fprintf(stderr, "ERROR: constructing dubins arc from lcm message dubins_primitive_t of wrong type %d\n",
        msg->type);
  }
}
DubinsArc::DubinsArc(const dubins::primitive_t * msg) :
    center(Map<const Vector2d>(msg->center)), R(msg->R), start_angle(bot_mod2pi(msg->start_angle)), end_angle(
        bot_mod2pi(msg->end_angle)), ccw(msg->ccw)
{
  this->sweep_angle = wrapTo2pi(this->ccw * (this->end_angle - this->start_angle));
  this->length = this->sweep_angle * this->R;
  this->type = this->ccw;
  this->curvature = this->ccw * (1.0 / this->R);

  if (msg->type != 1 && msg->type != -1) {
    fprintf(stderr, "ERROR: constructing dubins arc from lcm message dubins_primitive_t of wrong type %d\n",
        msg->type);
  }
}

double DubinsArc::getLength() const
{
  return this->length;
}

Eigen::Vector2d DubinsArc::getEndPoint() const
{
  return this->R * angleToVec(this->end_angle) + this->center;
}

Eigen::Vector2d DubinsArc::getStartPoint() const
{
  return this->R * angleToVec(this->start_angle) + this->center;
}

void DubinsArc::lcmgl_print(bot_lcmgl_t * lcmgl) const
    {
  bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
  double cur_angle = this->start_angle;
  Eigen::Vector2d cur_point;
  angleToVec(cur_angle, cur_point);
  cur_point *= this->R;
  cur_point += this->center;
  bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), 0);

  while ((cur_angle - this->start_angle) * this->ccw < this->sweep_angle) {
    cur_angle += M_PI / ARC_DRAW_RESOLUTION * this->ccw;
    angleToVec(cur_angle, cur_point);
    cur_point *= this->R;
    cur_point += this->center;
    bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), 0);
  }

  angleToVec(this->end_angle, cur_point);
  cur_point *= this->R;
  cur_point += this->center;
  bot_lcmgl_vertex3d(lcmgl, cur_point(0), cur_point(1), 0);
  bot_lcmgl_end(lcmgl);
}

Eigen::Vector3d DubinsArc::getStartPose() const
{
  Eigen::Vector3d pose;
  pose.block<2, 1>(0, 0) = this->getStartPoint();
  pose(2) = this->start_angle + M_PI / 2.0 * this->ccw;
  return pose;
}

Eigen::Vector3d DubinsArc::getEndPose() const
{
  Eigen::Vector3d pose;
  pose.block<2, 1>(0, 0) = this->getEndPoint();
  pose(2) = this->end_angle + M_PI / 2.0 * this->ccw;
  return pose;
}

template<typename T>
void _to_dubins_primitive_t(const DubinsArc* self, T * msg)
{
  msg->type = self->type;
  Eigen::Map<Vector2d>(msg->center) = self->center;
  msg->start_angle = self->start_angle;
  msg->ccw = self->ccw;
  msg->end_angle = self->end_angle;
  msg->R = self->R;
}
void DubinsArc::to_dubins_primitive_t(dubins_primitive_t * msg) const
    {
  _to_dubins_primitive_t(this, msg);
}
void DubinsArc::to_dubins_primitive_t(dubins::primitive_t * msg) const
    {
  _to_dubins_primitive_t(this, msg);
}

std::ostream& operator<<(std::ostream& output, const DubinsArc & arc)
{
  output << "Arc, center\n" << arc.center << "\nstart: " << arc.start_angle << " end: " << arc.end_angle << " sweep: "
      << arc.ccw * arc.sweep_angle << std::endl;
  return output;
}

double dubinsGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, DubinsPath & path)
{
  int opt_word[3];
  double length = dubinsGetDistance(R, xy_start, theta_start, xy_end, theta_end, opt_word);
  dubinsGetPathWord(R, xy_start, theta_start, xy_end, theta_end, path, opt_word);
  return length;
}

double dubinsGetPath(double R, const Eigen::Vector3d & x_start, const Eigen::Vector3d & x_end,
    DubinsPath & path)
{
  return dubinsGetPath(R, x_start.block<2, 1>(0, 0), x_start(2), x_end.block<2, 1>(0, 0), x_end(2), path);
}

void dubinsRemoveZeroLength(DubinsPath & path)
{

  DubinsPath::iterator it;
  for (it = path.begin(); it != path.end(); it++) {
    if (path.size() == 1)
      break; //prevent a zero length path

    DubinsPathPrimitive * cur_prim = *it;
    if (cur_prim->getLength() < 1e-6) {
      it = path.erase(it);
      delete cur_prim;
    }
  }
}

//dubinsGet the specific path with the sequence given by word
bool dubinsGetPathWord(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    double theta_end, DubinsPath & path, const int L[3])
{
  //make sure we have a valid L
  assert(L[0] == 1 || L[0] == -1);
  assert(L[1] <= 1 || L[1] >= -1);
  assert(L[2] == 1 || L[2] == -1);

  double start_arc_start_angle = theta_start - L[0] * M_PI / 2;
  double end_arc_end_angle = theta_end - L[2] * M_PI / 2;
  Eigen::Vector2d start_arc_center = xy_start - R * angleToVec(start_arc_start_angle);
  Eigen::Vector2d end_arc_center = xy_end - R * angleToVec(end_arc_end_angle);

  Eigen::Vector2d arc_center_to_center = end_arc_center - start_arc_center;
  double d_center_to_center = arc_center_to_center.norm();
  double angle_center_to_center = atan2Vec(arc_center_to_center);

  if (L[0] == L[2] && d_center_to_center < .0001) {
    DubinsArc * arc = new DubinsArc(start_arc_center, R, start_arc_start_angle, end_arc_end_angle, L[0]);
    path.push_back(arc);
    dubinsRemoveZeroLength(path);
    return true;
  }

  double end_arc_start_angle, start_arc_end_angle;

  if (L[1] != 0) {
    assert(L[0] == L[2]);
    assert(L[1] != L[0]);

    if (d_center_to_center > 4 * R) { //if the arc centers are more than 2 diameters apart, we can't make a path
      return false;
    }

    double start_end_sweep = acos(d_center_to_center / 2 / (2 * R));

    Eigen::Vector2d middle_arc_center;
    double middle_arc_start_angle, middle_arc_end_angle;

    DubinsArc * middle_arc[2];
    DubinsArc * start_arc[2];
    DubinsArc * end_arc[2];
    double lengths[2];

    int s[] = { 1, -1 };
    for (int ii = 0; ii < 2; ii++) {

      start_arc_end_angle = angle_center_to_center - s[ii] * L[0] * start_end_sweep;
      middle_arc_center = start_arc_center + 2 * R * angleToVec(start_arc_end_angle);
      middle_arc_start_angle = start_arc_end_angle + M_PI;
      end_arc_start_angle = angle_center_to_center + s[ii] * L[2] * start_end_sweep + M_PI;
      middle_arc_end_angle = end_arc_start_angle + M_PI;

      start_arc[ii] = new DubinsArc(start_arc_center, R, start_arc_start_angle, start_arc_end_angle, L[0]);
      middle_arc[ii] = new DubinsArc(middle_arc_center, R, middle_arc_start_angle, middle_arc_end_angle, L[1]);
      end_arc[ii] = new DubinsArc(end_arc_center, R, end_arc_start_angle, end_arc_end_angle, L[2]);

      lengths[ii] = start_arc[ii]->getLength() + middle_arc[ii]->getLength() + end_arc[ii]->getLength();
    }

    int use_ind, del_ind;
    if (lengths[0] < lengths[1]) {
      use_ind = 0;
      del_ind = 1;
    }
    else {
      use_ind = 1;
      del_ind = 0;
    }

    path.push_back(start_arc[use_ind]);
    path.push_back(middle_arc[use_ind]);
    path.push_back(end_arc[use_ind]);

    delete start_arc[del_ind];
    delete middle_arc[del_ind];
    delete end_arc[del_ind];

  }

  else {
    if (L[0] == L[2]) {

      start_arc_end_angle = angle_center_to_center - L[0] * M_PI_2;
      end_arc_start_angle = start_arc_end_angle;

    }
    else {

      //if the arcs overlap this won't work
      if (2 * R > d_center_to_center) {
        return false;
      }

      double radius_sum_angle = asin(2 * R / d_center_to_center);
      start_arc_end_angle = angle_center_to_center - L[0] * (M_PI_2 - radius_sum_angle);
      end_arc_start_angle = angle_center_to_center - L[2] * (M_PI_2 + radius_sum_angle);
    }

    DubinsArc * start_arc = new DubinsArc(start_arc_center, R, start_arc_start_angle, start_arc_end_angle, L[0]);
    DubinsArc * end_arc = new DubinsArc(end_arc_center, R, end_arc_start_angle, end_arc_end_angle, L[2]);

    Eigen::Vector2d line_start = start_arc->getEndPoint();
    Eigen::Vector2d line_end = end_arc->getStartPoint();
    DubinsLine * middle_line = new DubinsLine(line_start, line_end, start_arc->getEndPose()[2]);
    path.push_back(start_arc);
    path.push_back(middle_line);
    path.push_back(end_arc);

  }
  dubinsRemoveZeroLength(path);
//  int pl = path.size();
  return true;
}

double dubinsGetPath(double R, const Eigen::Vector2d & xy_start, double theta_start, const Eigen::Vector2d & xy_end,
    DubinsPath & path)
{

  Eigen::Vector2d start_to_end = xy_end - xy_start;
  double start_to_end_angle = atan2Vec(start_to_end);
  double heading_to_end = bot_mod2pi(start_to_end_angle - theta_start);
  int turn;
  if (heading_to_end >= 0) {
    turn = 1;
  }
  else {
    turn = -1;
  }

  double start_arc_start_angle = theta_start - turn * M_PI / 2;
  Eigen::Vector2d start_arc_center = xy_start - R * angleToVec(start_arc_start_angle);

  Eigen::Vector2d arc_center_to_end = xy_end - start_arc_center;
  double d_center_to_end = arc_center_to_end.norm();
  double angle_center_to_end = atan2Vec(arc_center_to_end);

  if (d_center_to_end < R) {
    return INFINITY;
  }

  double radius_sum_angle = asin(R / d_center_to_end);
  double start_arc_end_angle = angle_center_to_end - turn * (M_PI_2 - radius_sum_angle);

  DubinsArc * start_arc = new DubinsArc(start_arc_center, R, start_arc_start_angle, start_arc_end_angle, turn);

  Eigen::Vector2d line_start = start_arc->getEndPoint();
  DubinsLine * line = new DubinsLine(line_start, xy_end, start_arc->getEndPose()[2]);
  path.push_back(start_arc);
  path.push_back(line);

  dubinsRemoveZeroLength(path);
//  int pl = path.size();
  return dubinsGetPathLength(path);
}

double dubinsGetDistance(double R, const Eigen::Vector2d & xy_start, double theta_start,
    const Eigen::Vector2d & xy_end)
{
  DubinsPath path;
  double path_length = dubinsGetPath(R, xy_start, theta_start, xy_end, path);
  dubinsFreePath(path);
  return path_length;
}

double dubinsGetDistance(double R, const Eigen::Vector3d & xytheta_start, const Eigen::Vector2d & xy_end)
{
  return dubinsGetDistance(R, xytheta_start.block<2, 1>(0, 0), xytheta_start(2), xy_end);
}

double dubinsGetPath(double R, const Eigen::Vector3d & xytheta_start, const Eigen::Vector2d & xy_end,
    DubinsPath & path)
{

  Eigen::Vector2d xy_start = xytheta_start.topRows(2);
  double theta_start = xytheta_start(2);
  return dubinsGetPath(R, xy_start, theta_start, xy_end, path);
}

double dubinsGetDistanceWord(double R, const Eigen::Vector2d & xy_start, double theta_start,
    const Eigen::Vector2d & xy_end, double theta_end, const int L[3])
{
  DubinsPath path;
  if (dubinsGetPathWord(R, xy_start, theta_start, xy_end, theta_end, path, L)) {
    double path_length = dubinsGetPathLength(path);
    dubinsFreePath(path);
    return path_length;
  }
  else {
    return INFINITY;
  }
}

double dubinsGetDistance(double R, const Eigen::Vector2d & xy_start, double theta_start,
    const Eigen::Vector2d & xy_end, double theta_end, int L[3])
{

  int words[6][3] = { { -1, 1, -1 }, { 1, -1, 1 }, { 1, 0, 1 }, { -1, 0, -1 }, { -1, 0, 1 }, { 1, 0, -1 } };
  double lengths[6];
  int ii;

  int * opt_word;
  double opt_length = INFINITY;

  for (ii = 0; ii < 6; ii++) {
    lengths[ii] = dubinsGetDistanceWord(R, xy_start, theta_start, xy_end, theta_end, words[ii]);
    if (lengths[ii] < opt_length) {
      opt_length = lengths[ii];
      opt_word = words[ii];
    }
  }

  if (L != NULL) {
    L[0] = opt_word[0];
    L[1] = opt_word[1];
    L[2] = opt_word[2];
  }
  return opt_length;
}

double dubinsGetDistance(double R, const Eigen::Vector3d & x_start, const Eigen::Vector3d & x_end, int L[3])
{
  return dubinsGetDistance(R, x_start.block<2, 1>(0, 0), x_start(2), x_end.block<2, 1>(0, 0), x_end(2), L);
}

double dubinsAdvanceCarrot(const DubinsPath & path_data,
    DubinsPath::const_iterator & path_it, Eigen::Vector3d & x_carrot, int * step_ccw,
    double step_dl)
{

  if (path_it == path_data.end())
    return false;

  Eigen::Vector2d xy = x_carrot.block<2, 1>(0, 0);
  DubinsPathPrimitive * path_el = *path_it;

  if (path_el->type == 0) {
    DubinsLine * line = (DubinsLine *) path_el;
    double distance_left = (line->end_pt - xy).norm();
    assert(distance_left >= 0);
    *step_ccw = 0;
    if (distance_left < step_dl) {
      x_carrot = line->getEndPose();
      path_it++;
      double new_step_dl = step_dl - distance_left;
      return dubinsAdvanceCarrot(path_data, path_it, x_carrot, step_ccw, new_step_dl);
    }
    else {
      x_carrot.block<2, 1>(0, 0) += step_dl * line->unit_vec;
      return true;
    }
  }
  else {
    DubinsArc * arc = dynamic_cast<DubinsArc *>(path_el);
    double cur_arc_angle = x_carrot(2) - arc->ccw * (M_PI / 2.0);
    double distance_left = wrapTo2pi((arc->end_angle - cur_arc_angle) * arc->ccw) * arc->R;
    if (distance_left < 0) {
      std::cout << "distance_left:" << distance_left << "\n";
    }
    assert(distance_left >= 0);
    *step_ccw = arc->ccw;
    if (distance_left < step_dl) {
      x_carrot = arc->getEndPose();
      path_it++;
      double new_step_dl = step_dl - distance_left;
      return dubinsAdvanceCarrot(path_data, path_it, x_carrot, step_ccw, new_step_dl);
    }
    else {
      cur_arc_angle += step_dl / arc->R * arc->ccw;
      x_carrot.block<2, 1>(0, 0) << cos(cur_arc_angle) * arc->R, sin(cur_arc_angle) * arc->R;
      x_carrot.block<2, 1>(0, 0) += arc->center;
      x_carrot(2) = cur_arc_angle + M_PI / 2.0 * arc->ccw;
      return true;
    }
  }
}

//------------- Utility Functions for Dubins Paths ---------------------
double dubinsGetPathLength(const DubinsPath & path)
{
  double path_length = 0;
  DubinsPath::const_iterator path_it;
  for (path_it = path.begin(); path_it != path.end(); path_it++) {
    DubinsPathPrimitive * prim = *path_it;
    path_length += prim->getLength();
  }
  return path_length;
}

void dubinsFreePath(DubinsPath & path)
{
  DubinsPath::iterator path_it = path.begin();
  DubinsPathPrimitive * cur_prim;
  while (!path.empty()) {
    cur_prim = *path_it;
    delete cur_prim;
    path_it = path.erase(path_it);
  }
}

void dubins_lcmgl_printPath(const DubinsPath & path, bot_lcmgl_t * lcmgl)
{
  DubinsPath::const_iterator path_it;
  for (path_it = path.begin(); path_it != path.end(); path_it++) {
    (*path_it)->lcmgl_print(lcmgl);
  }

}

Eigen::ArrayXXd dubinsPathRasterize(const DubinsPath & path, double step_size)
{

  int numPoints = ceil(dubinsGetPathLength(path) / step_size) + 2;  //+2 for start and end
  Eigen::ArrayX3d points(numPoints, 3);

  int cnt = 0;
  //add front point
  Vector3d x_carrot = path.front()->getStartPose();
  points.row(cnt++) = x_carrot;
  DubinsPath::const_iterator path_it = path.begin();
  int step_ccw;
  while (dubinsAdvanceCarrot(path, path_it, x_carrot, &step_ccw, step_size)) {
    assert(cnt<numPoints);
    x_carrot(2) = bot_mod2pi(x_carrot(2));
    points.row(cnt++) = x_carrot;
  }

  //add end point
  if (cnt < numPoints && (points.row(cnt).matrix().transpose() - path.back()->getEndPose()).norm() > 1e-6) {
    points.row(cnt++) = path.back()->getEndPose();
  }
  points.conservativeResize(cnt, 3);
  return points;

}

dubins_primitive_list_t * to_dubins_primitive_list_t(const DubinsPath & path)
{
  dubins_primitive_list_t * msg = (dubins_primitive_list_t *) calloc(1, sizeof(dubins_primitive_list_t));
  msg->num_primitives = path.size();
  msg->path_list = (dubins_primitive_t *) calloc(msg->num_primitives, sizeof(dubins_primitive_t));

  DubinsPath::const_iterator path_it;
  int ii = 0;
  for (path_it = path.begin(); path_it != path.end(); path_it++) {
    (*path_it)->to_dubins_primitive_t(&msg->path_list[ii]);
    ii++;
  }
  return msg;
}

template<typename T>
void toDubinsPath_impl(const T * msg, DubinsPath & path)
{
  for (int ii = 0; ii < msg->num_primitives; ii++) {
    path.push_back(DubinsPathPrimitive::from_dubins_primitive_t(&msg->path_list[ii]));
  }
}

void toDubinsPath(const dubins_primitive_list_t * msg, DubinsPath & path)
{
  toDubinsPath_impl(msg, path);
}
void toDubinsPath(const dubins::primitive_list_t * msg, DubinsPath & path)
{
  toDubinsPath_impl(msg, path);
}

DubinsPathPrimitive * DubinsPathPrimitive::copy() const
{
  if (this->type == 0) {
    const DubinsLine * self = dynamic_cast<const DubinsLine*>(this);
    return new DubinsLine(*self);
  }
  else {
    const DubinsArc * self = dynamic_cast<const DubinsArc*>(this);
    return new DubinsArc(*self);
  }
}
