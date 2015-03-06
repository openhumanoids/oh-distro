#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <lcmtypes/bot_core.h>
#include <bot_core/lcm_util.h>
#include "kinectOpenniLCM.h"

typedef struct {
    lcm_t *lcm;
    KinectOpenniLCM * kinectOpenniLCM;
} app_t;

static void
app_run(app_t *self)
{
  GMainLoop *mainloop = g_main_loop_new(NULL, FALSE);
  bot_glib_mainloop_attach_lcm(self->lcm);
  bot_signal_pipe_glib_quit_on_kill(mainloop);
  
  g_main_loop_run(mainloop);
  
  printf("main: exiting\n");
  
  g_main_loop_unref(mainloop);
}

static void
app_destroy(app_t *self)
{
  delete self->kinectOpenniLCM;
  lcm_destroy(self->lcm);
  free(self);
}

int main(int argc, char **argv)
{
    setlinebuf(stdout);    

    g_thread_init(NULL);

    app_t *app = (app_t*) calloc(1, sizeof(app_t));
    app->lcm = bot_lcm_get_global(NULL);
    app->kinectOpenniLCM = new KinectOpenniLCM(argc, argv);

    app_run(app);
    app_destroy(app);

    return 0;
}
