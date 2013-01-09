#ifndef RENDERER_ROBOTPLAN_HPP
#define RENDERER_ROBOTPLAN_HPP

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

void setup_renderer_robot_plan(BotViewer *viewer, int render_priority, lcm_t *lcm);

#endif //RENDERER_ROBOTPLAN_HPP
