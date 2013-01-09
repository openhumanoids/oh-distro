#ifndef GL_DRAW_UTILS_HPP
#define GL_DRAW_UTILS_HPP


#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

namespace visualization_utils {

static void normalize(GLfloat *a) {
	GLfloat d=sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
	a[0]/=d; a[1]/=d; a[2]/=d;
}

static void drawtri(GLfloat *a, GLfloat *b, GLfloat *c, int div, float r) {
	if (div<=0) {
		glNormal3fv(a); glVertex3f(a[0]*r, a[1]*r, a[2]*r);
		glNormal3fv(b); glVertex3f(b[0]*r, b[1]*r, b[2]*r);
		glNormal3fv(c); glVertex3f(c[0]*r, c[1]*r, c[2]*r);
	} else {
		GLfloat ab[3], ac[3], bc[3];
		for (int i=0;i<3;i++) {
			ab[i]=(a[i]+b[i])/2;
			ac[i]=(a[i]+c[i])/2;
			bc[i]=(b[i]+c[i])/2;
		}
		normalize(ab); normalize(ac); normalize(bc);
		drawtri(a, ab, ac, div-1, r);
		drawtri(b, bc, ab, div-1, r);
		drawtri(c, ac, bc, div-1, r);
		drawtri(ab, bc, ac, div-1, r);
	}
}

/*
 * Draws a sphere using subdivisions. TODO: add static sphere vertices
 */
static void drawSphere(int ndiv, float radius) {
 double X = 0.525731112119133606;
 double Z = 0.850650808352039932;
  GLfloat vdata[12][3] = {    
		  {-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z},
		  {0.0, Z, X}, {0.0, Z, -X}, {0.0, -Z, X}, {0.0, -Z, -X},
		  {Z, X, 0.0}, {-Z, X, 0.0}, {Z, -X, 0.0}, {-Z, -X, 0.0}
  };
  GLuint tindices[20][3] = { 
		{0,4,1}, {0,9,4}, {9,5,4}, {4,5,8}, {4,8,1},
		{8,10,1}, {8,3,10}, {5,3,8}, {5,2,3}, {2,7,3},
		{7,10,3}, {7,6,10}, {7,11,6}, {11,0,6}, {0,1,6},
		{6,1,10}, {9,0,11}, {9,11,2}, {9,2,5}, {7,2,11} };

	glBegin(GL_TRIANGLES);
	for (int i=0;i<20;i++)
		drawtri(vdata[tindices[i][0]], vdata[tindices[i][1]], vdata[tindices[i][2]], ndiv, radius);
	glEnd();
}

} // end namespace 




#endif //GL_DRAW_UTILS_HPP
