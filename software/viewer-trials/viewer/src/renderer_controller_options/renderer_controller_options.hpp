#ifndef __renderer_controller_options_h__
#define ___renderer_controller_options_h__

#include <maps/ViewClient.hpp>
#include <maps/BotWrapper.hpp>

#include <lcm/lcm.h>

#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>


typedef struct _RendererControllerOptions {
  BotRenderer renderer;
  BotViewer *viewer;
  lcm_t *lc;

  BotGtkParamWidget *pw;
  int64_t robot_utime;
  int8_t map_command;
  
}RendererControllerOptions;

void setup_renderer_controller_options(BotViewer *viewer, int render_priority, lcm_t* lcm, BotParam * param,
    BotFrames * frames);

void publish_options(RendererControllerOptions* self);

/**
 * @}
 */


#endif
