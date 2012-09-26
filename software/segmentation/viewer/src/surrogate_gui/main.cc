#include <string.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <gtk/gtk.h>
#include <lcm/lcm.h>

/*#include <bot_vis/viewer.h>
#include <bot_vis/bot_vis.h>
#include <bot_vis/param_widget.h>
#include <bot_vis/glm.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
*/


#include "UIProcessing.h"
#include "LinearAlgebra.h"
#include "perception/PclSurrogateUtils.h"

using namespace surrogate_gui;

typedef struct {
    BotViewer *viewer;
    lcm_t *lcm;
} state_t;

/////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	//LinearAlgebra::runTests();
	//PclSurrogateUtils::runTests();
	//if (true) return 1;

	/*for (int i = 0; i < Color::HIGHLIGHT; i++)
	{
		cout << " i = " << i << " getColor(i) = " <<
				Color::getColor(i) << " | as int = " <<
				((int) Color::getColor(i)) << endl;
	}

	vector<Color::StockColor> colors = Color::getDifferentColors(Color::BLUE, 5);
	cout << endl << "===different colors w/ initial = " << Color::BLUE << "====" << endl;
	for (uint i = 0; i < colors.size(); i++)
	{
		cout << "\nnext color = " << colors[i] << endl;
	}


	return 0;
*/
    gtk_init(&argc, &argv);
    glutInit(&argc, argv);
    g_thread_init(NULL);

    setlinebuf(stdout);

    state_t app;
    memset(&app, 0, sizeof(app));

    BotViewer *viewer = bot_viewer_new("Viewer-Surrogate");
    app.viewer = viewer;
    app.lcm = lcm_create(NULL);
    bot_glib_mainloop_attach_lcm(app.lcm);

    // setup renderers
    bot_viewer_add_stock_renderer(viewer, BOT_VIEWER_STOCK_RENDERER_GRID, 1);
    //KinectRendererXYZRGB *krxyzrgb = kinect_add_renderer_xyzrgb_to_viewer(viewer, 0,NULL,NULL);

	// create segmentation handler
    std::string channel_name = "LOCAL_MAP_POINTS";
    UIProcessing uip(viewer, app.lcm, channel_name.c_str()); //"KINECT_XYZRGB_ROS");

    std::cout << "Listening to data from channel: " << channel_name << "\n";

    // load saved preferences
    char *fname = g_build_filename(g_get_user_config_dir(), ".surrogate_guirc", NULL);
    bot_viewer_load_preferences(viewer, fname);

    printf("\n\n\n========USAGE:\n");
    printf("'a' = display axes for currently force or rotation-axis vector [track mode]\n");
    printf("'c' = camera move\n");
    printf("'g' = display trajector determined by force vector + rotation axis\n");
    printf("'l' = display lcm cloud on/off\n");
    printf("'m' = model fit to current selection\n");
    printf("'r' = rectangle select (valid in selection mode)\n");
    printf("'s' = turn segment coloring on/off (valid in selection mode)\n");
    printf("'t' = turn tracking info on/off\n");
    printf("'u' = turn model fit display on/off in segmenting mode\n");
    printf("'<--' and '-->' switch between sub-components of an object [segment mode]");
    printf("'<--', '-->', [up|down] arrows +- [shift_l and/or shift_r]: rotate vector [track mode]");
    printf("'tab', switch between rotation axis vector and force vector [track mode]");

    printf("\n==============\n\n\n");

    // run the main loop
    gtk_main();

    // save any changed preferences
    bot_viewer_save_preferences(viewer, fname);
    free(fname);

    // cleanup
    bot_viewer_unref(viewer);

    return 0;
}
