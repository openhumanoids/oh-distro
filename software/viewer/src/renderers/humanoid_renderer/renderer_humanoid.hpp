#ifndef HUMANOID_RENDERER_H
#define HUMANOID_RENDERER_H

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

void setup_renderer_humanoid(BotViewer *viewer, int render_priority, lcm_t *lcm);
//=============================================
// DRAW CUBOID ROUTINES
//=============================================
//Cube points and colours

void polygon(int a, int b, int c , int d);
void cube(void); 

#endif //HUMANOID_RENDERER_H
