#ifndef __renderer_end_effector_goal_h__
#define __renderer_end_effector_goal_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-end-effector-goal/renderer_end-effector-goal.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-end-effector-goal`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


void setup_renderer_end_effector_goal(BotViewer *viewer, int render_priority, lcm_t* lcm);

/**
 * @}
 */


#endif
