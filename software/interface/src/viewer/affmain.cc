#include <sys/wait.h> //waitpid
#include <fstream>
#include <sstream>
#include <gui_widgets/AffordancePane.h>
#include <dirent.h> // list files in dir
#include <cstdlib>
#include <iostream>
#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_core/lcm_util.h>
#include <boost/signals.hpp>
#include <GL/glut.h>
#include <boost/shared_ptr.hpp>

#include <GL/glut.h>

#include "otdf_parser/otdf_parser.h"
#include "lcmtypes/drc_lcmtypes.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    boost::shared_ptr<otdf::ModelInterface> object;

      cout<<"IN AFFMAIN " << argc<<endl;

    if (argc >= 3 && strcmp(argv[1], "affordance") == 0) {
      // read the OTDF file and add the appropriate widgets
      cout << "parsing affordance " << argv[2] << endl;
      
      // read the file
      char* file_name_char = argv[2];
      std::string file_name = file_name_char;
      std::string dir_name = "/home/drc/drc/software/models/otdf/" + file_name;
      std::string xml_string;
      std::fstream xml_file(dir_name.c_str(), std::fstream::in);
      while ( xml_file.good() )
	{
	  std::string line;
	  std::getline( xml_file, line);
	  xml_string += (line + "\n");
	}
      xml_file.close();

	cout << xml_string.length() <<endl;

      object = otdf::parseOTDF(xml_string);
      if (!object){
	std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
	return -1;
      }
    } else if (argc == 1) {
        DIR *dpdf;
	struct dirent *epdf;
	dpdf = opendir("/home/drc/software/models/otdf/");
	if (dpdf != NULL){
	    int c = 0;
	    while (epdf = readdir(dpdf)){
	        //stringstream execStr;
	        //execStr << "./simple affordance " << epdf->d_name;
	        if (strcmp(epdf->d_name, ".") != 0 && strcmp(epdf->d_name, "..") != 0) {
		    cout << "spawning affordance pane for otdf " << epdf->d_name << endl;
		    pid_t pid = fork();
		    if (pid == 0) { // child
		        execl("./affmain", "affordance", epdf->d_name);
		    } else {
		      c++;
		    }
		}
	    }
	    for (int i = 0; i < c; i++) {
	      int exitstatus;
	      waitpid(-1, &exitstatus, 0);
	    }
	} else {
	  cout << "no affordances found" << endl;
	}
    } else {
      cout << "invalid command line arguments" << endl;
      exit(-1);
    }

	Gtk::Main kit(argc, argv);
	Gtk::GL::init(argc, argv);
	//glutInit(&argc, argv);
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
	g_thread_init(NULL); //no longer needed as of Glub 2.32
	
	lcm_t * lcm_c = bot_lcm_get_global(NULL);
	bot_glib_mainloop_attach_lcm(lcm_c);
	boost::shared_ptr<lcm::LCM> lcm_cpp = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm_c));
	AffordancePane window(lcm_cpp, object);

	//Shows the window and returns when it is closed.
	Gtk::Main::run(window);

	return 0;
}
