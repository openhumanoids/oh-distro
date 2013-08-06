#ifndef __multisense_renderer_h__
#define ___bot_renderer_h__

/**
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>


#ifdef __cplusplus
extern "C" {
#endif

void multisense_add_renderer_to_viewer(BotViewer* viewer, int priority,lcm_t* lcm, BotFrames * frames, 
                                       const char * camera_frame, const char * camera_channel, BotParam *param);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
