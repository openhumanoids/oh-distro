#ifndef _SelectionRectangle_hpp_
#define _SelectionRectangle_hpp_

#include <Eigen/Geometry>

typedef struct _GdkEventButton GdkEventButton;
typedef struct _GdkEventMotion GdkEventMotion;

namespace drc {

class SelectionRectangle {
public:
  SelectionRectangle();
  ~SelectionRectangle();

  void setColor(const Eigen::Vector3f& iColor);
  void setActive(const bool iVal);

  bool mousePress(const GdkEventButton* iEvent);
  bool mouseRelease(const GdkEventButton* iEvent);
  bool mouseMotion(const GdkEventMotion* iEvent);
  void draw();
  
protected:
  bool mActive;
  Eigen::Vector2f mDragPoint1;
  Eigen::Vector2f mDragPoint2;
  Eigen::Vector3f mColor;
};

}

#endif
