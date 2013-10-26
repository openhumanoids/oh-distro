#include "InteractiveBox.hpp"

#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>

using namespace maps;

InteractiveBox::
InteractiveBox() {
  mBoxPosition << 0,0,0;
  mBoxSize << 1,1,1;
  mBoxOrientation.setIdentity();
  composeBoxToWorld();

  mBoxPoints.resize(8);
  mBoxPoints[0] << -0.5,-0.5,-0.5;
  mBoxPoints[1] << +0.5,-0.5,-0.5;
  mBoxPoints[2] << +0.5,+0.5,-0.5;
  mBoxPoints[3] << -0.5,+0.5,-0.5;
  mBoxPoints[4] << -0.5,-0.5,+0.5;
  mBoxPoints[5] << +0.5,-0.5,+0.5;
  mBoxPoints[6] << +0.5,+0.5,+0.5;
  mBoxPoints[7] << -0.5,+0.5,+0.5;

  mBoxEdges.resize(12);
  mBoxEdges[0] << 0,1;
  mBoxEdges[1] << 1,2;
  mBoxEdges[2] << 2,3;
  mBoxEdges[3] << 3,0;
  mBoxEdges[4] << 4,5;
  mBoxEdges[5] << 5,6;
  mBoxEdges[6] << 6,7;
  mBoxEdges[7] << 7,4;
  mBoxEdges[8] << 0,4;
  mBoxEdges[9] << 1,5;
  mBoxEdges[10] << 2,6;
  mBoxEdges[11] << 3,7;

  mBoxPlanes.resize(6);
  mBoxPlanes[0] << -1, 0, 0, -0.5;
  mBoxPlanes[1] << +1, 0, 0, -0.5;
  mBoxPlanes[2] <<  0,-1, 0, -0.5;
  mBoxPlanes[3] <<  0,+1, 0, -0.5;
  mBoxPlanes[4] <<  0, 0,-1, -0.5;
  mBoxPlanes[5] <<  0, 0,+1, -0.5;

  // TODO: might need to reverse some of these
  mBoxQuads.resize(6);
  mBoxQuads[0] << 0,3,7,4;
  mBoxQuads[1] << 1,2,6,5;
  mBoxQuads[2] << 0,1,5,4;
  mBoxQuads[3] << 2,3,7,6;
  mBoxQuads[4] << 0,1,2,3;
  mBoxQuads[5] << 4,5,6,7;

  mBoxColor << 0,0,1;

  mBoxValid = false;
  mDragging = false;
  mWhichButton = 0;
}

InteractiveBox::
~InteractiveBox() {
}

void InteractiveBox::
drawBox() {
  if (!mBoxValid) return;

  // push all state
  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glPushClientAttrib(GL_ALL_ATTRIB_BITS);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  // set transform
  glMultMatrixf(mBoxToWorld.data());

  // grab transform
  GLdouble modelViewGl[16];
  GLdouble projectionGl[16];
  GLint viewportGl[4];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelViewGl);
  glGetDoublev(GL_PROJECTION_MATRIX, projectionGl);
  glGetIntegerv(GL_VIEWPORT, viewportGl);
  GLdouble x,y,z;
  gluProject(0,0,0,modelViewGl,projectionGl,viewportGl,&x,&y,&z);
  mBoxDepth = z;

  // draw box faces
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  //glEnable(GL_DEPTH_TEST);
  //glDepthFunc(GL_LESS);
  glEnable(GL_BLEND);
  glBegin(GL_QUADS);
  glColor4f(mBoxColor[0], mBoxColor[1], mBoxColor[2], 0.05);
  for (size_t i = 0; i < mBoxQuads.size(); ++i) {
    for (int j = 0; j < 4; ++j) {
      glVertex3fv(mBoxPoints[mBoxQuads[i][j]].data());
    }
  }
  glEnd();

  // draw box edges
  glColor3f(mBoxColor[0], mBoxColor[1], mBoxColor[2]);
  glLineWidth(1);
  glBegin(GL_LINES);
  for (size_t i = 0; i < mBoxEdges.size(); ++i) {
    glVertex3fv(mBoxPoints[mBoxEdges[i][0]].data());
    glVertex3fv(mBoxPoints[mBoxEdges[i][1]].data());
  }
  glEnd();

  // restore all state
  glPopMatrix();
  glPopClientAttrib();
  glPopAttrib();
}

void InteractiveBox::
composeBoxToWorld() {
  mBoxToWorld = Eigen::Translation3f(mBoxPosition) * mBoxOrientation *
    mBoxSize.asDiagonal();
}

int InteractiveBox::
whichFaceHit(const Eigen::Vector3f& iOrigin,
             const Eigen::Vector3f& iDirection,
             Eigen::Vector3f& oIntersection) {
  // transform ray
  Eigen::Affine3f worldToBox = mBoxToWorld.inverse();
  Eigen::Vector3f origin = worldToBox*iOrigin;
  Eigen::Vector3f direction = worldToBox.linear()*iDirection;
  Eigen::Vector3f intersection;

  // intersect with each face, record smallest distance
  float minDistance = 1e10;
  int bestFace = -1;
  for (size_t i = 0; i < mBoxPlanes.size(); ++i) {
    Eigen::Vector3f normal = mBoxPlanes[i].head<3>();
    float dot1 = normal.dot(direction);
    if (dot1 > 0) continue;
    float dist = -(mBoxPlanes[i][3] + normal.dot(origin)) / dot1;
    if (dist < minDistance) {
      intersection = origin + direction*dist;
      if ((intersection[0] < -0.51) || (intersection[0] > 0.51) ||
          (intersection[1] < -0.51) || (intersection[1] > 0.51) ||
          (intersection[2] < -0.51) || (intersection[2] > 0.51)) {
        continue;
      }
      minDistance = dist;
      bestFace = i;
    }
  }

  if (bestFace >= 0) {
    oIntersection = mBoxToWorld*intersection;
  }

  return bestFace;
}

void InteractiveBox::
setColor(const float iR, const float iG, const float iB) {
  mBoxColor << iR,iG,iB;
}

void InteractiveBox::
createBox(const Eigen::Vector3f& iPosition, const float iSize) {
  Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
  setBoxParameters(iPosition, Eigen::Vector3f(iSize,iSize,iSize), quat);
}

Eigen::Affine3f InteractiveBox::
getBoxToWorldTransform() const {
  return mBoxToWorld;
}

bool InteractiveBox::
getBoxParameters(Eigen::Vector3f& oPosition, Eigen::Vector3f& oSize,
                 Eigen::Quaternionf& oOrientation) const {
  if (!mBoxValid) return false;
  oPosition = mBoxPosition;
  oSize = mBoxSize;
  oOrientation = mBoxOrientation;
  return true;
}

void InteractiveBox::
setBoxParameters(const Eigen::Vector3f& iPosition,
                 const Eigen::Vector3f& iSize,
                 const Eigen::Quaternionf& iOrientation) {
  mBoxPosition = iPosition;
  mBoxSize = iSize;
  mBoxOrientation = iOrientation;
  composeBoxToWorld();
  mBoxValid = true;
}

void InteractiveBox::
setBoxValid(const bool iVal) {
  mBoxValid = iVal;
}

bool InteractiveBox::
getBoxValid() const {
  return mBoxValid;
}

bool InteractiveBox::
mousePress(const Eigen::Vector2f& iClickPoint, const int iButton,
           const Eigen::Vector3f& iOrigin,
           const Eigen::Vector3f& iDirection) {
  if (!mBoxValid) return false;

  Eigen::Vector3f pt;
  int whichFace = whichFaceHit(iOrigin, iDirection, pt);
  if (whichFace < 0) return false;

  mCurrentFace = whichFace;
  mDragging = true;
  mDragPoint1 = mDragPoint2 = iClickPoint;
  mBoxPositionBase = mBoxPosition;
  mBoxOrientationBase = mBoxOrientation;
  mBoxSizeBase = mBoxSize;
  
  switch (iButton) {
  case 1: mTransformMode = TransformModeTranslation; break;
  case 2: mTransformMode = TransformModeRotation; break;
  case 3: mTransformMode = TransformModeScale; break;
  default: break;
  }

  return true;
}

bool InteractiveBox::
mouseMotion(const Eigen::Vector2f& iCurPoint, const int iButtonMask,
            const Eigen::Vector3f& iOrigin,
            const Eigen::Vector3f& iDirection) {
  if (!mBoxValid) return false;
  if (!mDragging) return false;

  GLdouble modelViewGl[16];
  GLdouble projectionGl[16];
  GLint viewportGl[4];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelViewGl);
  glGetDoublev(GL_PROJECTION_MATRIX, projectionGl);
  glGetIntegerv(GL_VIEWPORT, viewportGl);

  mDragPoint2 = iCurPoint;

  // translate box
  if (mTransformMode == TransformModeTranslation) {
    Eigen::Vector3f transVector;
    GLdouble x1,y1,z1,x2,y2,z2;
    gluUnProject(mDragPoint1[0], viewportGl[3]-mDragPoint1[1], mBoxDepth,
                 modelViewGl, projectionGl, viewportGl, &x1, &y1, &z1);
    gluUnProject(mDragPoint2[0], viewportGl[3]-mDragPoint2[1], mBoxDepth,
                 modelViewGl, projectionGl, viewportGl, &x2, &y2, &z2);
    Eigen::Vector3f trans(x2-x1, y2-y1, z2-z1);
    mBoxPosition = mBoxPositionBase + trans;
    composeBoxToWorld();
  }

  // scale box
  else if (mTransformMode == TransformModeScale) {
    Eigen::Vector3f transVector;
    GLdouble x1,y1,z1,x2,y2,z2;
    gluUnProject(mDragPoint1[0], viewportGl[3]-mDragPoint1[1], mBoxDepth,
                 modelViewGl, projectionGl, viewportGl, &x1, &y1, &z1);
    gluUnProject(mDragPoint2[0], viewportGl[3]-mDragPoint2[1], mBoxDepth,
                 modelViewGl, projectionGl, viewportGl, &x2, &y2, &z2);
    Eigen::Vector3f trans(x2-x1, y2-y1, z2-z1);
    Eigen::Vector3f normal = mBoxPlanes[mCurrentFace].head<3>(); 
    normal = mBoxOrientation*normal;
    float distance = trans.dot(normal);
    int idx = mCurrentFace/2;
    const float minSize = 0.05;
    if (mBoxSizeBase[idx] + distance < minSize) {
      distance = minSize-mBoxSizeBase[idx];
    }
    mBoxSize[idx] = mBoxSizeBase[idx] + distance;
    mBoxPosition = mBoxPositionBase + normal*distance/2;
    composeBoxToWorld();
  }

  // rotate box
  else if (mTransformMode == TransformModeRotation) {
    Eigen::Vector3f ray1, ray2;
    double x,y,z;
    gluUnProject(mDragPoint1[0], viewportGl[3]-mDragPoint1[1], 1,
                 modelViewGl, projectionGl, viewportGl, &x,&y,&z);
    ray1 << Eigen::Vector3f(x,y,z).normalized();
    gluUnProject(mDragPoint2[0], viewportGl[3]-mDragPoint2[1], 1,
                 modelViewGl, projectionGl, viewportGl, &x,&y,&z);
    ray2 << Eigen::Vector3f(x,y,z).normalized();
    float theta = acos(ray1.dot(ray2));
    Eigen::Vector3f axis = ray2.cross(ray1).normalized();
    mBoxOrientation = Eigen::AngleAxisf(theta,axis)*mBoxOrientationBase;
    composeBoxToWorld();
  }

  return true;
}

bool InteractiveBox::
mouseRelease(const Eigen::Vector2f& iCurPoint, const int iButton,
             const Eigen::Vector3f& iOrigin,
             const Eigen::Vector3f& iDirection) {
  if (!mBoxValid) return false;
  mDragging = false;
  return true;
}
