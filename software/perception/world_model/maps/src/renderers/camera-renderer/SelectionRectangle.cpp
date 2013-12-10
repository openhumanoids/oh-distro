#include "SelectionRectangle.hpp"

#include <GL/gl.h>

using namespace drc;

SelectionRectangle::
SelectionRectangle() {
  mColor << 1,0,0;
  mDragPoint1 << 0,0;
  mDragPoint2 << 0,0;
  mActive = false;
}

SelectionRectangle::
~SelectionRectangle() {
  // TODO
}

void SelectionRectangle::
setColor(const Eigen::Vector3f& iColor) {
  mColor = iColor;
}

void SelectionRectangle::
setActive(const bool iVal) {
  mActive = iVal;
}

bool SelectionRectangle::
mousePress(const GdkEventButton* iEvent) {
  if (!mActive) return false;
  // TODO
  return false;
}

bool SelectionRectangle::
mouseRelease(const GdkEventButton* iEvent) {
  if (!mActive) return false;
  // TODO
  return false;
}

bool SelectionRectangle::
mouseMotion(const GdkEventMotion* iEvent) {
  if (!mActive) return false;
  // TODO
  return false;
}


void SelectionRectangle::
draw() {
  if (!mActive) return;

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  // TODO: glOrtho2D

  float x1(mDragPoint1[0]), y1(mDragPoint1[1]);
  float x2(mDragPoint2[0]), y2(mDragPoint2[1]);
  if (x1 > x2) std::swap(x1,x2);
  if (y1 > y2) std::swap(y1,y2);

  glColor3f(mColor[0], mColor[1], mColor[2]);
  glBegin(GL_LINE_LOOP);
  glVertex2f(x1,y1);
  glVertex2f(x2,y1);
  glVertex2f(x2,y2);
  glVertex2f(x1,y2);
  glEnd();

  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
}
