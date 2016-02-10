//#include "octomap_util.hpp"
#include <iostream>
#include <sstream>
#include <lcmtypes/octomap_utils.h>
#include <ConciseArgs>

#include "octomap_utils/octomap_util.hpp"

using namespace std;
using namespace octomap;
using namespace occ_map;

int main(int argc, char ** argv)
{

  ConciseArgs opt_parse(argc, argv, "map_file_name", "loads an octomap and publishes it to lcm");

  double repeat_period = -1;
  bool blurred_map = false;
  std::string channel = "OCTOMAP";

  opt_parse.add(repeat_period, "r", "repeat", "repeat period for lcm publishing, negative only publish once", false);
  opt_parse.add(blurred_map, "b", "blurred",
      "map file contains a blurred map, load with loadOctomap instead of default", false);
  opt_parse.add(channel, "c", "channel", "channel on which to transmit", false);

  string octomap_fname;
  opt_parse.parse(octomap_fname);

  lcm_t * lcm = lcm_create(NULL);

  printf("loading octomap from: %s\n", octomap_fname.c_str());

  octomap::ColorOcTree * tree;
//  if (blurred_map) {
//    double min_neg_log_like;
//    tree = octomap_utils::loadOctomap(octomap_fname.c_str(), &min_neg_log_like);
//    printf("loaded using loadOctomap, min_neg_log_like = %f\n", min_neg_log_like);
//  }
//  else {
//    tree = new OcTree(octomap_fname);
//  }
  AbstractOcTree* readTreeAbstract = AbstractOcTree::read(octomap_fname);
  tree = dynamic_cast<ColorOcTree*>(readTreeAbstract);


  double minX, minY, minZ, maxX, maxY, maxZ;
  tree->getMetricMin(minX, minY, minZ);
  tree->getMetricMax(maxX, maxY, maxZ);
  printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]  res: %f\n", minX, minY, minZ, maxX, maxY, maxZ,
      tree->getResolution());

  octomap_raw_t msg;
  msg.utime = bot_timestamp_now();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      msg.transform[i][j] = 0;
    }
    msg.transform[i][i] = 1;
  }

  std::cout << "dfsdfsd\n";

  std::stringstream datastream;
  tree->write(datastream);
  std::string datastring = datastream.str();
  msg.data = (uint8_t *) datastring.c_str();
  msg.length = datastring.size();

  octomap_raw_t_publish(lcm, channel.c_str(), &msg);
  if (repeat_period > 0) {
    while (1) {
      usleep(1e6 * repeat_period);
      fprintf(stderr, ".");
      octomap_raw_t_publish(lcm, channel.c_str(), &msg);
    }
  }
  fprintf(stderr, "done! \n");

  return 0;
}

