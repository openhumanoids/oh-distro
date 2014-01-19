#ifndef MAV_MAPPING_RENDERERS_H_
#define MAV_MAPPING_RENDERERS_H_
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#ifdef __cplusplus
extern "C" {
#endif

  void mav_mapping_point_cloud_reg_add_renderer_to_viewer(BotViewer *viewer, int render_priority);

#ifdef __cplusplus
}
#endif

#endif
