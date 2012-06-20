#ifndef __scrollingplots_bot_renderer_h__
#define ___bot_renderer_h__

/**
 * @defgroup scrollingplotsBotRenderer scrollingplotsBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include scrollingplots-renderer/scrollingplots_renderer.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs scrollingplots-renderer`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void scrollingplots_add_renderer_to_viewer(BotViewer* viewer, int priority,lcm_t* lcm);
//, BotFrames * frames, const char * scrollingplots_frame);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
