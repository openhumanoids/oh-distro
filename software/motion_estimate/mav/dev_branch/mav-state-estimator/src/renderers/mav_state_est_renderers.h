#ifndef __MAV_STATE_EST_RENDERERS_H__
#define __MAV_STATE_EST_RENDERERS_H__
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <bot_core/bot_core.h>

#ifdef __cplusplus
extern "C" {
#endif

void add_map_measurement_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm, BotParam * param,
    BotFrames * frames);

void add_mav_state_est_renderer_to_viewer(BotViewer *viewer, int render_priority, lcm_t * lcm, BotParam * param,
    BotFrames * frames);

#ifdef __cplusplus
}
#endif

#endif
