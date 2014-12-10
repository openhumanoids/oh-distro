#ifndef __rwx_bot_renderer_h__
#define __rwx_bot_renderer_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include scrollingplots-renderer/rwx_renderer.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs scrollingplots-renderer`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

void setup_renderer_rwx(BotViewer *viewer, int render_priority, const char *rwx_fname);

#endif
