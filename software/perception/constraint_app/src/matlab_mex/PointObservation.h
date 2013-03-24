#include <map>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

class PointObservation {
 public:
  PointObservation(const KDL::Vector& ob) : obs_expressedIn_world(ob) {}
  KDL::Vector obs_expressedIn_world;
};
