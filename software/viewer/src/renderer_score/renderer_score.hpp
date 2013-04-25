#ifndef RENDERER_SCORE_HPP_
#define RENDERER_SCORE_HPP_

/**
 * Linking: `pkg-config --libs renderer_status`
 * @{
 */

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>

void score_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm);

#endif /* RENDERER_SCORE_HPP_ */

