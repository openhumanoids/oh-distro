#ifndef COLLECTIONS_RENDERER_HPP_
#define COLLECTIONS_RENDERER_HPP_

/**
 * Linking: `pkg-config --libs collections_render`
 * @{
 */

#include <lcm/lcm.h>
#include <bot_vis/bot_vis.h>

void collections_add_renderer_to_viewer(BotViewer *viewer, int render_priority);

#endif /* COLLECTIONS_RENDERER_HPP_ */

