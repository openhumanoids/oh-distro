#include <string.h>
#include <stdlib.h>
#include <gtk/gtk.h>
#include <iostream>

#include <bot_vis/bot_vis.h>
#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

//renderers
#include <renderer_cam_thumb_drc/renderer_cam_thumb_drc.h>

using namespace std;


// TBD - correct location for this code?
static void on_top_view_clicked(GtkToggleToolButton *tb, void *user_data)
{
  BotViewer *self = (BotViewer*) user_data;

  double eye[3];
  double look[3];
  double up[3];
  self->view_handler->get_eye_look(self->view_handler, eye, look, up);
  eye[0] = 0;
  eye[1] = 0;
  eye[2] = 1;
  look[0] = 0;
  look[1] = 2;
  look[2] = 0;
  up[0] = 0;
  up[1] = 4;
  up[2] = 0;
  self->view_handler->set_look_at(self->view_handler, eye, look, up);

  bot_viewer_request_redraw(self);
}


int main(int argc, char *argv[])
{
  gtk_init(&argc, &argv);
  glutInit(&argc, argv);
  g_thread_init(NULL);

//  if (argc < 2) {
//    fprintf(stderr, "usage: %s <render_plugins>\n", g_path_get_basename(argv[0]));
//    exit(1);
//  }
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_glib_mainloop_attach_lcm(lcm);
  BotParam * bot_param;

    bot_param = bot_param_new_from_server(lcm, 0);
    if (bot_param == NULL) {
      fprintf(stderr, "Couldn't get bot param from server.\n");
      return 1;
    }

  BotFrames* bot_frames = bot_frames_new(lcm, bot_param);


  BotViewer* viewer = bot_viewer_new("Camera Viewer");
  //die cleanly for control-c etc :-)
  bot_gtk_quit_on_interrupt();

  // setup renderers
  bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
  add_cam_thumb_drc_renderer_to_viewer(viewer, 0, lcm, bot_param, bot_frames);

  // add custom TOP VIEW button
  GtkWidget *top_view_button;
  top_view_button = (GtkWidget *) gtk_tool_button_new_from_stock(GTK_STOCK_ZOOM_FIT);
  gtk_tool_button_set_label(GTK_TOOL_BUTTON(top_view_button), "Top View");
  gtk_tool_item_set_tooltip(GTK_TOOL_ITEM(top_view_button), viewer->tips, "Switch to Top View", NULL);
  gtk_toolbar_insert(GTK_TOOLBAR(viewer->toolbar), GTK_TOOL_ITEM(top_view_button), 4);
  gtk_widget_show(top_view_button);
  g_signal_connect(G_OBJECT(top_view_button), "clicked", G_CALLBACK(on_top_view_clicked), viewer);
  on_top_view_clicked(NULL, (void *) viewer);  

  //load the renderer params from the config file.
  char *fname = g_build_filename(g_get_user_config_dir(), ".bot-plugin-viewerrc_camera", NULL);
  bot_viewer_load_preferences(viewer, fname);

  gtk_main();

  //save the renderer params to the config file.
  bot_viewer_save_preferences(viewer, fname);

  bot_viewer_unref(viewer);
}
