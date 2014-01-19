/**
 * @file isam.cpp
 * @brief Main isam program.
 * @author Michael Kaess
 * @version $Id: isam.cpp 3364 2010-11-11 16:17:30Z kaess $
 *
 * Copyright (C) 2009-2010 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson and John J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <iostream>

const std::string usage =
    "\n"
    "Usage:\n"
    "  iSAM [OPTION...] FILE\n"
    "\n"
    "Options:\n"
    "  -h  -?       show help options\n"
    "  -v           verbose - additional output\n"
    "  -q           quiet - no output\n"
    "  -n <number>  max. number of lines to read, 0=all\n"
    "  -G           GUI: show in 3D viewer\n"
    "  -L           LCM: send data to external process\n"
    "  -S [fname]   save statistics\n"
    "  -W [fname]   write out final result\n"
    "  -F           force use of numerical derivatives\n"
    "  -C           calculate marginal covariances\n"
    "  -B           batch processing\n"
    "  -N           no optimization\n"
    "  -d <number>  #steps between drawing/sending data\n"
    "  -u <number>  #steps between any updates (batch or incremental)\n"
    "  -b <number>  #steps between batch steps, 0=never\n"
    "  -s <number>  #steps between solution (backsubstitution)\n"
    "\n";

#include <stdio.h>
#include <cstring>
#include <map>

#include <isam/isam.h>

#include "Loader.h"
#ifdef USE_LCM
#include "Lcm.h"
#endif
#ifdef USE_GUI
// needed in file with main() function to work on Mac...
#include "SDL.h"
#include "Viewer.h"
#endif

// for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

using namespace std;
using namespace isam;

const int FNAME_MAX = 500;
char fname[FNAME_MAX];
char fname_stats[FNAME_MAX] = "isam_stats.txt";
char fname_result[FNAME_MAX] = "isam_result.txt";

bool use_gui = false;
bool use_lcm = false;
bool save_stats = false;
bool write_result = false;
bool calculate_covariances = false;
bool batch_processing = false;
bool no_optimization = false;
int parse_num_lines = 0;

// draw state every mod_draw steps
int mod_draw = 1;

Slam slam;
Properties prop;
#ifdef USE_LCM
Lcm lcm;
bool lcm_first = true;
#endif
#ifdef USE_GUI
Viewer viewer;
#endif
Loader* loader;

// for recording of statistics for each time step (for visualization)
class Stats {
public:
  double time;
  double chi2;
  unsigned int nnz;
  unsigned int nnodes;
  unsigned int nconstraints;
};
vector<class Stats> stats;

/**
 * Command line argument processing.
 */
void process_arguments(int argc, char* argv[]) {
  int c;
  while ((c = getopt(argc, argv, ":h?vqn:GLS:W:CBFNd:u:b:s:") ) != -1) {
    // Each option character has to be in the string in getopt();
    // the first colon changes the error character from '?' to ':';
    // a colon after an option means that there is an extra
    // parameter to this option; 'W' is a reserved character
    switch (c) {
    case 'h':
    case '?':
      cout << usage;
      exit(0);
      break;
    case 'v':
      prop.verbose = true;
      break;
    case 'q':
      prop.quiet = true;
      prop.verbose = false;
      break;
    case 'n':
      parse_num_lines = atoi(optarg);
      require(parse_num_lines>0, "Number of lines (-n) must be positive (>0).");
      break;
    case 'G':
#ifndef USE_GUI
      require(false, "GUI support (-G) was disabled at compile time")
#endif
      use_gui = true;
      break;
    case 'L':
#ifndef USE_LCM
      require(false, "LCM support (-L) was disabled at compile time")
#endif
      use_lcm = true;
      break;
    case 'S':
      save_stats = true;
      if (optarg!=NULL) {
        strncpy(fname_stats, optarg, FNAME_MAX);
      }
      break;
    case 'W':
      write_result = true;
      if (optarg!=NULL) {
        strncpy(fname_result, optarg, FNAME_MAX);
      }
      break;
    case 'C':
      calculate_covariances = true;
      break;
    case 'B':
      batch_processing = true;
      break;
    case 'F':
      prop.force_numerical_jacobian = true;
      break;
    case 'N':
      no_optimization = true;
      break;
    case 'd':
      mod_draw = atoi(optarg);
      require(mod_draw>0, "Number of steps between drawing (-d) must be positive (>0).");
      break;
    case 'u':
      prop.mod_update = atoi(optarg);
      require(prop.mod_update>0, "Number of steps between updates (-u) must be positive (>0).");
      break;
    case 'b':
      prop.mod_batch = atoi(optarg);
      require(prop.mod_batch>=0, "Number of steps between batch steps (-b) must be positive or zero (>=0).");
      break;
    case 's':
      prop.mod_solve = atoi(optarg);
      require(prop.mod_solve>0, "Number of steps between solving (-s) must be positive (>0).");
      break;
    case ':': // unknown option, from getopt
      cout << usage;
      exit(1);
      break;
    }
  }

  if (argc > optind+1) {
    cout << endl;
    cout << "Error: Too many arguments." << endl;
    cout << usage;
    exit(1);
  } else if (argc == optind+1) {
    strncpy(fname, argv[optind], FNAME_MAX);
  } else {
    cout << endl;
    cout << "Error: Too few arguments." << endl;
    cout << usage;
    exit(1);
  }

}

/**
 * Save statistics for each time step for external visualization.
 */
void save_statistics(const string& fname) {
  ofstream out(fname.c_str(), ios::out | ios::binary);
  require(out, "Cannot open statistics file.");
  for (unsigned int i=0; i<stats.size(); i++) {
    out << i << " " << stats[i].time << " " << stats[i].chi2 << " " << stats[i].nnz;
    out << " " << stats[i].nconstraints << " " << stats[i].nnodes;
    out << endl;
  }
  out.close();
}

/**
 * Calculate covariances for both points and poses up to the given time step.
 */
void covariances(unsigned int step, list<Matrix>& point_marginals,
                 list<Matrix>& pose_marginals) {
  // make sure return arguments are empty
  point_marginals.clear();
  pose_marginals.clear();

  // combining everything into one call is faster,
  // as it avoids recalculating commonly needed entries
  Slam::node_lists_t node_lists;
  for (unsigned int i=0; i<step; i++) {
    list<Node*> entry;
    entry.push_back(loader->pose_nodes()[i]);
    node_lists.push_back(entry);
  }
  for (unsigned int i=0; i<loader->num_points(step); i++) {
    list<Node*> entry;
    entry.push_back(loader->point_nodes()[i]);
    node_lists.push_back(entry);
  }
  pose_marginals = slam.marginal_covariance(node_lists);

  // split into points and poses
  if (pose_marginals.size() > 0) {
    list<Matrix>::iterator center = pose_marginals.begin();
    for (unsigned int i=0; i<step; i++, center++);
    point_marginals.splice(point_marginals.begin(), pose_marginals,
                           center, pose_marginals.end());
  }
}

/**
 * Visualize data during optimization in internal viewer
 * or send data via LCM (to an external viewer).
 */
void visualize(unsigned int step) {
  list<Matrix> point_marginals;
  list<Matrix> pose_marginals;
  if (calculate_covariances && (use_gui || use_lcm) && (step%mod_draw==0)) {
    covariances(step, point_marginals, pose_marginals);
  }

#ifdef USE_LCM
  {
    // ids also determine color in viewer
    const int id_trajectory = 0;
    const int id_landmarks = 1;
    const int id_constraints = 2;
    const int id_measurements = 3;
    const int id_pose_covs = 4;
    const int id_point_covs = 5;

    // send to LCM viewer
    if (use_lcm && lcm_first) {
      lcm.send_reset();
      lcm_first = false;
    }
    if (use_lcm && step%mod_draw==0) {
      lcm.send_nodes(loader->poses(step), id_trajectory, (char*)"Trajectory", 1);
      lcm.send_nodes(loader->points(step), id_landmarks, (char*)"Landmarks", 2);
      lcm.send_links(loader->constraints(step), id_constraints,
          (char*)"Odometry", id_trajectory, id_trajectory);
      lcm.send_links(loader->measurements(step), id_measurements,
          (char*)"Measurements", id_trajectory, id_landmarks);
      if (calculate_covariances) {
        lcm.send_covariances(pose_marginals,  id_pose_covs,
            (char*)"Pose Covs",  id_trajectory, loader->is_3d());
        lcm.send_covariances(point_marginals, id_point_covs,
            (char*)"Point Covs", id_landmarks,  loader->is_3d());
      }
    }
  }
#endif

#ifdef USE_GUI
  {
    // display in internal 3D viewer
    const int id_trajectory = 0;
    const int id_landmarks = 1;
    const int id_constraints = 2;
    const int id_measurements = 3;
    const int id_pose_covs = 4;
    const int id_point_covs = 5;
    if (use_gui && step%mod_draw==0) {
      viewer.set_nodes(loader->poses(step), id_trajectory,
          (char*)"Trajectory", VIEWER_OBJ_POSE3D);
      viewer.set_nodes(loader->points(step), id_landmarks,
          (char*)"Landmarks", VIEWER_OBJ_TREE);
      viewer.set_links(loader->constraints(step), id_constraints,
          "Odometry", id_trajectory, id_trajectory);
      viewer.set_links(loader->measurements(step), id_measurements,
          "Measurements", id_trajectory, id_landmarks);
      if (calculate_covariances) {
        viewer.set_covariances(pose_marginals,  id_pose_covs,
            (char*)"Pose Covs",  id_trajectory, loader->is_3d());
        viewer.set_covariances(point_marginals, id_point_covs,
            (char*)"Point Covs", id_landmarks,  loader->is_3d());
      }
    }
  }
#endif
}

/**
 * Quit if viewer was closed.
 */
void check_quit() {
#ifdef USE_GUI
  if (viewer.exit_requested()) {
    cout << endl << "Aborted by user..." << endl;
    exit(0);
  }
#endif
}

/**
 * Incrementally process factors.
 */
void incremental_slam() {
  unsigned int step = 0;

    for (; step<loader->num_steps(); step++) {

      check_quit();

      double t0 = tic();

      tic("setup");

      // add new variables and constraints
      for(list<Node*>::const_iterator it = loader->nodes(step).begin();
          it!=loader->nodes(step).end(); it++) {
        if (prop.verbose) cout << **it << endl;
        slam.add_node(*it);
      }
      for(list<Factor*>::const_iterator it = loader->factors(step).begin();
          it!=loader->factors(step).end(); it++) {
        if (prop.verbose) cout << **it << endl;
        slam.add_factor(*it);
      }

      toc("setup");
      tic("incremental");

      if (!(batch_processing || no_optimization)) {
        slam.update();
      }

      toc("incremental");

      if (save_stats) {
        stats.resize(step+1);
        stats[step].time = toc(t0);
        stats[step].chi2 = slam.normalized_chi2();
        stats[step].nnz = slam.get_R().nnz();
        stats[step].nnodes = slam.get_nodes().size();
        stats[step].nconstraints = slam.get_factors().size();
      }

      // visualization is not counted in timing
      if (!(batch_processing || no_optimization)) {
        visualize(step);
      }
    }

  visualize(step-1);

  if (!no_optimization) {
    if (batch_processing) {
      tic("batch");
      slam.batch_optimization();
      toc("batch");
    } else {
      // end with a batch step/relinearization
      prop.mod_batch = 1;
      slam.set_properties(prop);
      tic("final");
      slam.update();
      toc("final");
    }
  }

  visualize(step-1);
}

/**
 * The actual processing of data, in separate thread if GUI enabled.
 */
int process(void* unused) {

  // incrementally process data
  slam.set_properties(prop);
  incremental_slam();

  toc("all");

  if (!prop.quiet) {
    if (!batch_processing) {
      cout << endl;
    }
    double accumulated = tictoc("setup") + tictoc("incremental") + tictoc("batch");
    cout << "Accumulated computation time: " << accumulated << "s" << endl;
    cout << "(Overall execution time: " << tictoc("all") << "s)" << endl;
    slam.print_stats();
    cout << endl;
  }

  if (save_stats) {
    cout << "Saving statistics to " << fname_stats << endl;
    save_statistics(fname_stats);
    cout << endl;
  }
  if (write_result) {
    cout << "Saving result to " << fname_result << endl;
    slam.save(fname_result);
    cout << endl;
  }


#ifdef USE_GUI
  if (use_gui) {
    while (true) {
      if (viewer.exit_requested()) {
        exit(0);
      }
      SDL_Delay(100);
    }
  }
#endif

  exit(0);
}

/**
 * Everything starts here.
 */
int main(int argc, char* argv[]) {

  tic("all");

  cout << endl;
  cout << "Incremental Smoothing and Mapping (iSAM) version 1.5" << endl;
  cout << "(C) 2009-2010 Massachusetts Institute of Technology" << endl;
  cout << "Michael Kaess, Hordur Johannsson, and John J. Leonard" << endl;
  cout << endl;

  process_arguments(argc, argv);

  if (!prop.quiet) {
    cout << "Reading " << fname;
    if (parse_num_lines>0) {
      cout << " (only " << parse_num_lines << " lines)";
    }
    cout << endl;
  }
  // parse all data and get into suitable format for incremental processing
  Loader loader_(fname, parse_num_lines, prop.verbose);
  loader = &loader_;
  if (!prop.quiet) {
    loader_.print_stats();
    cout << endl;
  }

  if (!prop.quiet) {
    if (batch_processing) {
      cout << "Performing SAM (batch processing)\n";
    } else {
      cout << "Performing iSAM with parameters:\n";
      cout << "  Draw/send every " << mod_draw << " steps\n";
      cout << "  Update every " << prop.mod_update << " steps\n";
      cout << "  Solve every " << prop.mod_solve << " steps\n";
      cout << "  Batch every " << prop.mod_batch << " steps\n";
    }
    cout << endl;
  }

#ifdef USE_GUI
  if (use_gui) {
    cout << "3D viewer:\n";
    cout << "  Exit: Esc or \"q\"\n";
    cout << "  Reset view: \"r\"\n";
    cout << "  Toggle background color \"c\"\n";
    cout << "  Rotate: left mouse button\n";
    cout << "  Translate: middle mouse button or CTRL+left mouse button\n";
    cout << "  Scale: right mouse button or SHIFT+left mouse button or mouse wheel\n";
    cout << endl;
    // process will be run in separate thread
    viewer.init(process);
  } else {
    process(NULL);
  }
#else
  // no threads needed, simply run process
  process(NULL);
#endif

  return 0;
}
