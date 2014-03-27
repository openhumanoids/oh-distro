#include "map_measurement_function.hpp"
#include <path_util/path_util.h>
#include <ConciseArgs>

using namespace std;
using namespace MavStateEst;

int main(int argc, char * argv[])
{

  double mPP = 1;
  double degPP = 24;
  double deg_phi_max = 46; //max phi-roll angle in pixelmap
  double sigma_prior = 1; //m in xyz cov
  double z_height = 3; //m
  double vis_pause = -1;
  double max_laser_range = 15.0;

  ConciseArgs opt_parse(argc, argv, "map_file_name",
      "generates a map measurement function using simulated laser and gpf (settings in state est)");
  opt_parse.add(mPP, "m", "mPP", "meters per pixel resolution", false);
  opt_parse.add(degPP, "d", "dPP", "degrees per pixel resolution", false);
  opt_parse.add(deg_phi_max, "p", "phi_max", "max roll angle", false);
  opt_parse.add(sigma_prior, "s", "sigma", "sigma prior in meters", false);
  opt_parse.add(z_height, "z", "height", "planning height", false);
  opt_parse.add(vis_pause, "v", "vis_pause", "visualize gpf pause time, negative turns off visualization", false);
  opt_parse.add(max_laser_range, "l", "max_laser", "maximum laser range", false);

  string base_filename;
  opt_parse.parse(base_filename);

  lcm_t * lcm = bot_lcm_get_global(NULL);
  BotParam * param = bot_param_get_global(lcm, 0);
  if (param == NULL) {
    char param_fname[1024];
    sprintf(param_fname, "%s/fixie.cfg", getConfigPath());
    fprintf(stderr, "Warning: No param server running, falling back to param file:\n %s\n", param_fname);
    param = bot_param_new_from_file(param_fname);
  }
  BotFrames * frames = bot_frames_get_global(lcm, param);

  MapMeasurementFunction map_meas;

  map_meas.generateFromOctomap(base_filename.c_str(), mPP, bot_to_radians(deg_phi_max), bot_to_radians(degPP),
      lcm, param, frames, pow(sigma_prior, 2), z_height, vis_pause, max_laser_range);

  char save_filename[1024];
  sprintf(save_filename, "%s_meas_map%d", base_filename.c_str(), int(max_laser_range));
  map_meas.saveToFile(save_filename);
  map_meas.publishInformationMaps(lcm);
  map_meas.publishMapMeasurementFunction(lcm);
  fprintf(stdout, "\n\npublished summary pixelmaps and saved measurement map as %s\n", save_filename);
//
//  sleep(10);
//  fprintf(stdout, "\n\njust to test, load maps, and republishing\n");
//  MapMeasurementFunction loaded;
//  loaded.loadFromFile(save_filename);
//  loaded.publishInformationMaps(lcm);
}
