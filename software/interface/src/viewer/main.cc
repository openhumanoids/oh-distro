#include <sys/wait.h> //waitpid
#include <fstream>
#include <sstream>
#include <dirent.h> // list files in dir
#include <cstdlib>
#include <iostream>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/lcm_util.h>
#include <boost/signals.hpp>
#include <GL/glut.h>
#include <gui_widgets/MainPane.h>
#include <boost/shared_ptr.hpp>

#include <GL/glut.h>

#include "otdf_parser/otdf_parser.h"
#include "lcmtypes/drc_lcmtypes.hpp"

using namespace std;

int main(int argc, char *argv[])
{
	Gtk::Main kit(argc, argv);
	Gtk::GL::init(argc, argv);
	glutInit(&argc, argv);
	// we can't simply fork as X will bork

	//
	// Query OpenGL extension version.
	//
	int major, minor;
		Gdk::GL::query_version(major, minor);
		std::cout << "OpenGL extension version - "
			  << major << "." << minor << std::endl;

	//
	// Instantiate and run the application.
	//
	//g_thread_init(NULL); no longer needed as of Glub 2.32
	
	lcm_t * lcm_c = bot_lcm_get_global(NULL);
	bot_glib_mainloop_attach_lcm(lcm_c);
	boost::shared_ptr<lcm::LCM> lcm_cpp = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm_c));
	gui::MainPane window(lcm_cpp);

	//Shows the window and returns when it is closed.
	Gtk::Main::run(window);

	return 0;
}
