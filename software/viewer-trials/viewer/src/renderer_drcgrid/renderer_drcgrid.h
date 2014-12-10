#ifndef __drcgrid_bot_renderer_h__
#define __drcgrid_bot_renderer_h__

/**
 * Linking: `pkg-config --libs drcgrid-renderer`
 * @{
 */

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>
#include <visualization_utils/keyboard_signal_utils.hpp>

void drcgrid_add_renderer_to_viewer(BotViewer* viewer, int priority,lcm_t* lcm, visualization_utils::KeyboardSignalRef signalRef);


#endif
