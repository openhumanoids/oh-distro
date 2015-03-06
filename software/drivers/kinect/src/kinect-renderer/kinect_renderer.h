#ifndef __kinect_bot_renderer_h__
#define ___bot_renderer_h__

/**
 * @defgroup KinectBotRenderer KinectBotRenderer renderer
 * @brief BotVis Viewer renderer plugin
 * @include kinect-renderer/kinect_renderer.h
 *
 * TODO
 *
 * Linking: `pkg-config --libs kinect-renderer`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>


#ifdef __cplusplus
extern "C" {
#endif

void kinect_add_renderer_to_viewer(BotViewer* viewer, int priority,lcm_t* lcm, BotFrames * frames, const char * kinect_frame, BotParam *param);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
