#ifndef __renderer_manipulation_h__
#define ___renderer_manipulation_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-manipulation/renderer_manipulation.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-manipulation`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


void setup_renderer_manipulation(BotViewer *viewer, int render_priority, lcm_t* lcm);

/**
 * @}
 */


#endif
