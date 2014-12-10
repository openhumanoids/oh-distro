#ifndef __renderer_localize_h__
#define ___renderer_localize_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-localize/renderer_localize.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-localize`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


void setup_renderer_localize(BotViewer *viewer, int render_priority, lcm_t* lcm);

/**
 * @}
 */


#endif
