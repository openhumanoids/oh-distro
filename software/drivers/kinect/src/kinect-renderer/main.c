#include <string.h>

#include <gtk/gtk.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
//#include <er_common/path_util.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>

#include "kinect_renderer.h"

typedef struct {
    BotViewer *viewer;
    lcm_t *lcm;
} state_t;

/////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    gtk_init(&argc, &argv);
    glutInit(&argc, argv);
    g_thread_init(NULL);

    setlinebuf(stdout);

    state_t app;
    memset(&app, 0, sizeof(app));

    BotViewer *viewer = bot_viewer_new("Viewer");
    app.viewer = viewer;
    app.lcm = lcm_create(NULL);
    bot_glib_mainloop_attach_lcm(app.lcm);

    // setup calibration params
    BotParam * param = NULL;

    // setup renderers
    bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
    kinect_add_renderer_to_viewer(viewer, 0,app.lcm,NULL,"Kinect", param);

    // load saved preferences
    char *fname = g_build_filename(g_get_user_config_dir(), ".kinect-viewerrc", NULL);
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
