#include "map_measurement_function.hpp"

using namespace std;
using namespace MavStateEst;

int main(int argc, char ** argv)
{

  if (argc <2) {
    printf("Usage:\n");
    printf("%s <map_meas_fname> [repeat_period]\n", argv[0]);
    exit(1);
  }

  string map_fname = argv[1];

  double repeat_period = -1;
  if (argc > 2)
    repeat_period = atof(argv[2]);

  lcm_t * lcm = lcm_create(NULL);

  printf("loading map meas from: %s\n", map_fname.c_str());

  MapMeasurementFunction map_meas;

  map_meas.loadFromFile(map_fname.c_str());

  map_meas.publishMapMeasurementFunction(lcm);
  if (repeat_period > 0) {
    while (1) {
      usleep(1e6 * repeat_period);
      fprintf(stderr,".");
      map_meas.publishMapMeasurementFunction(lcm);
    }
  }
  fprintf(stderr, "done! \n");

  return 0;
}

