#ifndef __renderer_otdf_h__
#define __renderer_otdf_h__


#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#define POINT3D(p) (&(((union _point3d_any_t *)(p))->point))


void setup_renderer_otdf(BotViewer *viewer, int render_priority, lcm_t* lcm);

/**
 * @}
 */


#endif
