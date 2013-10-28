#ifndef CAM_THUMB_RENDERER_H_
#define CAM_THUMB_RENDERER_H_
#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

void add_cam_thumb_drc_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm, BotParam * param,
    BotFrames * frames);

#ifdef __cplusplus
}
#endif

#endif
