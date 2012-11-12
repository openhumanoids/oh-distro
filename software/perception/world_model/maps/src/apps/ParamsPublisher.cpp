#include <string>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/drc/map_params_t.hpp>
#include <bot_core/timestamp.h>

#include <ConciseArgs>

using namespace std;

int main(const int iArgc, const char** iArgv) {
  string channel = "MAP_CREATE";
  int id(1);
  double res(0.02);
  double xdim(10), ydim(10), zdim(10);
  ConciseArgs opt(iArgc, (char**)iArgv);
  opt.add(channel, "c", "channel", "channel to publish create message");
  opt.add(id, "i", "id", "id of new map");
  opt.add(res, "r", "res", "resolution of new map");
  opt.add(xdim, "x", "xdim", "x size of new map");
  opt.add(ydim, "y", "ydim", "y size of new map");
  opt.add(zdim, "z", "zdim", "z size of new map");
  opt.parse();

  drc::map_params_t msg;
  msg.utime = bot_timestamp_now();
  msg.message_id = 0;
  msg.map_id = id;
  msg.resolution = res;
  msg.dimensions[0] = xdim;
  msg.dimensions[1] = ydim;
  msg.dimensions[2] = zdim;
  msg.transform_to_local.translation.x = 0;
  msg.transform_to_local.translation.y = 0;
  msg.transform_to_local.translation.z = 0;
  msg.transform_to_local.rotation.x = 0;
  msg.transform_to_local.rotation.y = 0;
  msg.transform_to_local.rotation.z = 0;
  msg.transform_to_local.rotation.w = 1;
  lcm::LCM theLcm;
  theLcm.publish(channel, &msg);

  return 0;
}
