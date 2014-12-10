#ifndef __renderer_driving_h__
#define ___renderer_driving_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-driving/renderer_driving.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-driving`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))

void setup_renderer_driving(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames);

/**
 * @}
 */


#endif
