#ifndef __renderer_navigation_h__
#define ___renderer_navigation_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-navigation/renderer_navigation.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-navigation`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


void setup_renderer_navigation(BotViewer *viewer, int render_priority, lcm_t* lcm);

/**
 * @}
 */


#endif
