#ifndef PANORAMA_RENDERER_HPP_
#define PANORAMA_RENDERER_HPP_

/**
 * Linking: `pkg-config --libs panorama_render`
 * @{
 */

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>

void panorama_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm);

#endif /* PANORAMA_RENDERER_HPP_ */

