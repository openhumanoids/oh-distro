#ifndef _maps_InteractiveBox_hpp_
#define _maps_InteractiveBox_hpp_

#include <vector>
#include <Eigen/Geometry>

namespace maps {

class InteractiveBox {
protected:
  enum TransformMode {
    TransformModeTranslation,
    TransformModeScale,
    TransformModeRotation,
  };

public:
  InteractiveBox();
  ~InteractiveBox();

  void drawBox();

  bool mousePress(const Eigen::Vector2f& iClickPoint, const int iButton,
                  const Eigen::Vector3f& iOrigin,
                  const Eigen::Vector3f& iDirection);
  bool mouseMotion(const Eigen::Vector2f& iCurPoint, const int iButtonMask,
                   const Eigen::Vector3f& iOrigin,
                   const Eigen::Vector3f& iDirection);
  bool mouseRelease(const Eigen::Vector2f& iCurPoint, const int iButton,
                    const Eigen::Vector3f& iOrigin,
                    const Eigen::Vector3f& iDirection);

  void setColor(const float iR, const float iG, const float iB);
  void createBox(const Eigen::Vector3f& iPosition, const float iSize);

  Eigen::Affine3f getBoxToWorldTransform() const;
  void setBoxParameters(const Eigen::Vector3f& iPosition,
                        const Eigen::Vector3f& iSize,
                        const Eigen::Quaternionf& iOrientation);
  bool getBoxParameters(Eigen::Vector3f& oPosition, Eigen::Vector3f& oSize,
                        Eigen::Quaternionf& oOrientation) const;
  bool getBoxValid() const;
  void setBoxValid(const bool iVal);

protected:
  int whichFaceHit(const Eigen::Vector3f& iOrigin,
                   const Eigen::Vector3f& iDirection,
                   Eigen::Vector3f& oIntersection);
  void composeBoxToWorld();

protected:
  Eigen::Vector3f mBoxPosition;
  Eigen::Vector3f mBoxSize;
  Eigen::Quaternionf mBoxOrientation;
  Eigen::Affine3f mBoxToWorld;
  std::vector<Eigen::Vector3f> mBoxPoints;
  std::vector<Eigen::Vector4f> mBoxPlanes;
  std::vector<Eigen::Vector4i> mBoxQuads;
  std::vector<Eigen::Vector2i> mBoxEdges;

  bool mBoxValid;
  bool mDragging;
  int mWhichButton;
  Eigen::Vector2f mDragPoint1;
  Eigen::Vector2f mDragPoint2;
  TransformMode mTransformMode;
  int mCurrentFace;
  Eigen::Vector3f mBoxPositionBase;
  Eigen::Vector3f mBoxSizeBase;
  Eigen::Quaternionf mBoxOrientationBase;
  float mBoxDepth;

  Eigen::Vector3f mBoxColor;
};

}

#endif
