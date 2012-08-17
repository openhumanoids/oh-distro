#ifndef __fovis_bot2_visualization_hpp__
#define __fovis_bot2_visualization_hpp__

#include <bot_lcmgl_client/lcmgl.h>

namespace fovis
{

class VisualOdometry;
class StereoCalibration;

class Visualization {
public:
  Visualization(bot_lcmgl_t* lcmgl, const StereoCalibration* calib);
  virtual ~Visualization();

  void draw(const VisualOdometry* odom);
  void draw_pyramid_level_flow(const VisualOdometry* odom, int level_num);
  void draw_pyramid_level_matches(const VisualOdometry* odom, int level_num);

private:
  Visualization (const Visualization& other);
  Visualization& operator=(const Visualization& other);

  void colormap(float z, float rgb[3]);

  bot_lcmgl_t* _lcmgl;
  const StereoCalibration* _calibration;
  float _min_z;
  float _max_z;
};

}

#endif
