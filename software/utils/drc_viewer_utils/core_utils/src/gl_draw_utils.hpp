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


inline static void draw_axis(double x, double y,double z, 
                      double yaw, double pitch, double roll, 
                      double size, bool mark)
{
    glPushMatrix();
    glPushAttrib(GL_CURRENT_BIT);

    glTranslatef(x, y, z);

    glRotatef(bot_to_degrees(yaw),  0., 0., 1.);
    glRotatef(bot_to_degrees(pitch),0., 1., 0.);
    glRotatef(bot_to_degrees(roll), 1., 0., 0.);

    glBegin(GL_LINES);
    glColor3f(1.0,0.0,0.0); glVertex3f(0.0,0.0,0.0);glVertex3f(size*1.0,0.0,0.0);
    glColor3f(0.0,1.0,0.0); glVertex3f(0.0,0.0,0.0);glVertex3f(0.0,size*1.0,0.0);
    glColor3f(0.0,0.0,1.0); glVertex3f(0.0,0.0,0.0);glVertex3f(0.0,0.0,size*1.0);
    glEnd();

    if (mark) {
        //    glutWireSphere(size*1.5, 5, 5);
    }

    glPopAttrib();
    // todo: reset color?
    glPopMatrix();
}

} // end namespace 




#endif //GL_DRAW_UTILS_HPP
