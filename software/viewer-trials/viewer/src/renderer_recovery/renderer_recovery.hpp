#ifndef __renderer_recovery_h__
#define ___renderer_recovery_h__

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

void setup_renderer_recovery(BotViewer *viewer, int render_priority, lcm_t* lcm, BotParam * param,
    BotFrames * frames);

#endif
