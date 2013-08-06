#include <iostream>
#include <vector>
#include <string.h>

#include <gtk/gtk.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include <bot_frames/bot_frames_renderers.h>
#include "multisense_renderer.h"
#include <bot_lcmgl_render/lcmgl_bot_renderer.h>

/////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    std::cout << "Launch: multisense-viewer bot_config_file.cfg (otherwise listen to server)\n";
    std::string config_file = "";
    if (argc > 1){
      config_file = argv[1];
    }
  
    gtk_init(&argc, &argv);
    glutInit(&argc, argv);
    g_thread_init(NULL);

    setlinebuf(stdout);

    lcm_t * lcm;
    lcm= lcm_create(NULL);    

    BotViewer *viewer = bot_viewer_new("Multisense Viewer");

    bot_glib_mainloop_attach_lcm(lcm);
    
    // setup calibration params
    BotParam * bot_param;
    if(config_file.size()) {
      fprintf(stderr,"Reading config from file\n");
      bot_param = bot_param_new_from_file(config_file.c_str());
      if (bot_param == NULL) {
        std::cerr << "Couldn't get bot param from file %s\n" << config_file << std::endl;
        exit(-1);
      }
    }else {
      bot_param = bot_param_new_from_server(lcm, 0);
      if (bot_param == NULL) {
        fprintf(stderr, "Couldn't get bot param from server.\n");
        return 1;
      }
    }
    BotFrames* bot_frames = bot_frames_new(lcm, bot_param);
    
    // setup renderers
    bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
    // to view lidar:
    bot_lcmgl_add_renderer_to_viewer(viewer, lcm, 1);
    // to view camera:
    multisense_add_renderer_to_viewer(viewer, 0,lcm,bot_frames,"CAMERA", "CAMERA", bot_param);
    
    multisense_add_renderer_to_viewer(viewer, 0,lcm,NULL,"CAMERA_FUSED", "CAMERA_FUSED", bot_param);

    // load saved preferences
    char *fname = g_build_filename(g_get_user_config_dir(), ".multisense-viewerrc", NULL);
    bot_viewer_load_preferences(viewer, fname);

    // run the main loop
    gtk_main();

    // save any changed preferences
    bot_viewer_save_preferences(viewer, fname);
    free(fname);

    // cleanup
    bot_viewer_unref(viewer);

    return 0;
}
