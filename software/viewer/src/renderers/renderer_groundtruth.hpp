#ifndef __renderer_groundtruth_h__
#define ___renderer_groundtruth_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include renderer-groundtruth/renderer_groundtruth.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs renderer-groundtruth`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


void setup_renderer_groundtruth(BotViewer *viewer, int render_priority, lcm_t* lcm);

/**
 * @}
 */


#endif
