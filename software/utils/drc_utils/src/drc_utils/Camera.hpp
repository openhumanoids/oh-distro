#ifndef _drc_Camera_hpp_
#define _drc_Camera_hpp_

#include <memory>
#include <Eigen/Geometry>

typedef struct _BotParam BotParam;
typedef struct _BotCamTrans BotCamTrans;

namespace drc {
class Camera {
public:
  typedef std::shared_ptr<Camera> Ptr;

public:
  Camera(BotParam* iBotParam, const std::string& iName);
  Camera(BotCamTrans* iCamTrans, const bool iShouldFree=true);
  virtual ~Camera();

  Eigen::Matrix3d getCalibrationMatrix() const;
  Eigen::Vector2d project(const Eigen::Vector3d& iPoint) const;
  Eigen::Vector3d unproject(const Eigen::Vector2d& iPixel) const;

protected:
  struct Helper;
  std::shared_ptr<Helper> mHelper;
};
}

#endif
