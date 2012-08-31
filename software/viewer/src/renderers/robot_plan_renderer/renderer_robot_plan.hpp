#ifndef ROBOT_PLAN_RENDERER_H
#define ROBOT_PLAN_RENDERER_H

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

void setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm);

#endif //ROBOT_PLAN_RENDERER_H
