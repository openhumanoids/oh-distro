#ifndef RENDERER_ROBOT_STATE_HPP
#define RENDERER_ROBOT_STATE_HPP

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>

void setup_renderer_robot_state(BotViewer *viewer, int render_priority, lcm_t *lcm);


#endif //RENDERER_ROBOT_STATE_HPP
