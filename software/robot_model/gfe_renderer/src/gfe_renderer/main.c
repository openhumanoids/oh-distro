#include <string.h>
#include <stdlib.h>

#include <gtk/gtk.h>

#include <bot_vis/bot_vis.h>

void setup_gfe_renderer(BotViewer *viewer, int render_priority, 
                              const char *wavefront_fname0,const char *wavefront_fname1);

int main(int argc, char *argv[])
{
    gtk_init(&argc, &argv);
    glutInit(&argc, argv);
    g_thread_init(NULL);

    if(argc < 2) {
        fprintf(stderr, "usage: %s <obj_filename>\n", 
                g_path_get_basename(argv[0]));
        exit(1);
    }

    const char *wavefront_fname0 = argv[1];
    const char *wavefront_fname1 = argv[2];
    
    BotViewer* viewer = bot_viewer_new("Wavefront OBJ Viewer");

    char *fname = g_build_filename(g_get_user_config_dir(), ".wavefront-viewerrc", NULL);
    
    bot_viewer_load_preferences(viewer, fname);

    // setup renderers
    bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
    setup_gfe_renderer(viewer, 1, wavefront_fname0,wavefront_fname1);

    gtk_main();

    bot_viewer_save_preferences(viewer, fname);

    bot_viewer_unref(viewer);
}
