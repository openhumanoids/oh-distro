#include <octomap_utils/octomap_util.hpp>
#include <bot_core/bot_core.h>
#include <sstream>
using namespace std;
using namespace octomap;
using namespace occ_map;

int main(int argc, char ** argv)
{

  double blur_sigma = .5;
  if (argc > 2) {
    printf("Usage:\n");
    printf("%s <blur_radius>\n", argv[0]);
    exit(1);
  }
  if (argc > 2)
    blur_sigma = atof(argv[2]);

  string octomap_fname = "z_plane.bt";
  std::stringstream s;
  s << octomap_fname << "_blurred_" << blur_sigma;
  string blurred_fname = s.str();
  double z_plane_height = 0;
  double resolution = .1;

  double xy0[2] = { -100, -100 }; //min bound;
  double xy1[2] = { 100, 100 }; //max bound;

  octomap::OcTree * ocTree = octomap_utils::createZPlane(resolution, xy0, xy1, z_plane_height);
  printf("generated initial oct Tree\n");

  double minX, minY, minZ, maxX, maxY, maxZ;
  ocTree->getMetricMin(minX, minY, minZ);
  ocTree->getMetricMax(maxX, maxY, maxZ);
  printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]  res: %f\n", minX, minY, minZ, maxX, maxY, maxZ,
      ocTree->getResolution());

  printf("blurring octomap\n");
  double minNegLogLike;
  octomap::OcTree * ocTree_blurred = octomap_utils::octomapBlur(ocTree, blur_sigma, &minNegLogLike);

  fprintf(stderr, "Saving Unblurred map to  %s...", octomap_fname.c_str());
  ocTree->writeBinaryConst(octomap_fname.c_str());
  fprintf(stderr, "done! \n");

  fprintf(stderr, "Saving blurred map to: %s\n", blurred_fname.c_str());
  octomap_utils::saveOctomap(ocTree_blurred, blurred_fname.c_str(), minNegLogLike);

  fprintf(stderr, "all done!\n");
  return 0;
}
