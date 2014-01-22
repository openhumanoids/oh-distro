#define EIGEN_DONT_ALIGN
#define _BT_DEBUG
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "../belief_tree.hpp"
#include "../drawing_defs.hpp"
#include "../environment_defs.hpp"
#include "../lqg_planning.hpp"
#include "../dubins_beacon.cpp"
#include <getopt.h>
#include <bot_core/bot_core.h>
#include <lcm/lcm.h>
#include <vector>
#include <eigen_utils/eigen_utils.hpp>

//default RRBT algorithm settings
#define DEF_BALL_RADIUS_CONSTANT 500.0
#define DEF_MAX_BALL_RADIUS 10.0

#define DEF_DUB_BALL_RADIUS_CONSTANT 1000000.0
#define DEF_DUB_MAX_BALL_RADIUS 18.0

#define DEF_EPSILON 1.0/10.0
#define DEF_NUM_SAMPLES 100
#define DEF_NUM_SIM_OPT_PATH 1000
#define DEF_ENVIRONMENT 'p'
#define DEF_OUTPUT_FILENAME "output_rrbt"
#define DEF_NSIGMA 3.0

//default sleep constants
#define DEF_ANIMATE_USLEEP 50000
#define DEF_DRAW_MOD 1

using namespace Eigen;
using namespace std;

BeliefTreeInterface * getBeliefTree(char sys_type, double ball_radius_constant, double max_ball_radius, double epsilon,
    int draw_mod, bt_visualization_t draw_animate, int animate_usleep, bool RRBT_mode, double nsigma,
    search_t search_order)
{
  switch (sys_type) {
  case 'p':
    ;
    {
      //pathological example where you have to go backwards then forwards
      //LQG properties
      //convenience identity and 0 matrices

      //system types
      const int N = 2;
      typedef Eigen::Matrix<double, N, 1> X;
      typedef Eigen::Matrix<double, N, N> StateCov;
      typedef Line<N> P;
      typedef LQGAug<N> A;

      LQGBoxSystem<N> * dyn_sys = new LQGBoxSystem<2> (nsigma);

      StateCov I = StateCov::Identity();
      StateCov Z = StateCov::Zero();

      dyn_sys->dxdt = .25;

      //base LQG properties (all others are I)
      dyn_sys->lqg_base.K = .1 * I;
      dyn_sys->lqg_base.R = 1000000 * I;
      dyn_sys->lqg_base.Q = .001 * I;

      //initial conditions
      StateCov Sigma_init = I;
      StateCov Lambda_init = Z;
      dyn_sys->aug_init = LQGAug<N> (Sigma_init, Lambda_init, 0);
      dyn_sys->x_init << 3.5, 18.5;

      //environment setup
      X env_center;
      env_center << 10.0, 15.0;
      X env_size;
      env_size << 20.0, 30.0;

      RegionCSpace<N> * region_cspace = new RegionCSpace<N> (new HyperBox<N> (env_center, env_size));

      //obstacles
      X left_obstacle_center;
      left_obstacle_center << 4.0, 23.0;
      X right_obstacle_center;
      right_obstacle_center << 16.0, 23.0;
      X obstacle_size;
      obstacle_size << 8.0, 2.0;

      region_cspace->addObstacle(new HyperBox<N> (left_obstacle_center, obstacle_size));
      region_cspace->addObstacle(new HyperBox<N> (right_obstacle_center, obstacle_size));

      dyn_sys->c_space = region_cspace;

      //goal
      X goal_center;
      goal_center << 18.0, 28.0;
      X goal_size;
      goal_size << 4.0, 4.0;
      dyn_sys->goal_region = new HyperBox<N> (goal_center, goal_size);

      //information centers
      X map_center;
      map_center << 10.0, 4.0;
      X map_size;
      map_size << 20.0, 8.0;

      LQGProperties<N> map_properties = dyn_sys->lqg_base;
      map_properties.R = I * .01;
      dyn_sys->lqg_regions.push_back(new LQGHyperCubeRegion<N> (map_properties, HyperBox<N> (map_center, map_size)));

      BeliefTreeKD<N, P, A> * rrbt = new BeliefTreeKD<N, P, A> (dyn_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);

      return rrbt;
    }
  case 'h':
    ;
    {
      //homogenous environment where the covariances are initiated at steady state
      //convenience identity and 0 matrices

      //system types
      const int N = 2;
      typedef Eigen::Matrix<double, N, 1> X;
      typedef Eigen::Matrix<double, N, N> StateCov;
      typedef Line<N> P;
      typedef LQGAug<N> A;

      LQGBoxSystem<N> * dyn_sys = new LQGBoxSystem<2> (nsigma);

      StateCov I = StateCov::Identity();
      StateCov Z = StateCov::Zero();

      dyn_sys->dxdt = .25;

      //base LQG properties (all others are I)
      dyn_sys->lqg_base.K = .4 * I;
      dyn_sys->lqg_base.R = .5 * I;
      dyn_sys->lqg_base.Q = .5 * I;

      //initial conditions
      StateCov Sigma_init = 0.309017 * I;
      StateCov Lambda_init = 0.78125 * I;
      dyn_sys->aug_init = LQGAug<N> (Sigma_init, Lambda_init, 0);
      dyn_sys->x_init << 5, 5;

      //environment setup
      X env_center;
      env_center << 10.0, 10.0;
      X env_size;
      env_size << 20.0, 20.0;

      RegionCSpace<N> * region_cspace = new RegionCSpace<N> (new HyperBox<N> (env_center, env_size));
      X obs_center;
      obs_center << 11, 10;
      region_cspace->addObstacle(new Circle(obs_center, 2));
      obs_center << 5, 15;
      region_cspace->addObstacle(new Circle(obs_center, 4));
      obs_center << 16, 15;
      region_cspace->addObstacle(new Circle(obs_center, 1));

      dyn_sys->c_space = region_cspace;

      //goal
      X goal_center;
      goal_center << 18.0, 18.0;
      dyn_sys->goal_region = new Circle(goal_center, 2);

      BeliefTreeKD<N, P, A> * rrbt = new BeliefTreeKD<N, P, A> (dyn_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);
      //      BeliefTreeKD<N, P, A> * rrbt = new BeliefTreeKD<N, P, A> (dyn_sys, ball_radius_constant, max_ball_radius, epsilon,
      //          draw_mod, draw_animate, animate_usleep);

      return rrbt;
    }

  case 'd':
    ;
    {

      if (ball_radius_constant == DEF_BALL_RADIUS_CONSTANT) {
        ball_radius_constant = DEF_DUB_BALL_RADIUS_CONSTANT;
      }

      if (max_ball_radius == DEF_MAX_BALL_RADIUS) {
        max_ball_radius = DEF_DUB_MAX_BALL_RADIUS;
      }

      //Dubins_beacon environment
      typedef DubinsBeacon::X X;
      typedef DubinsBeacon::StateCov StateCov;
      typedef DubinsBeacon::P P;
      typedef DubinsBeacon::A A;

      double sigma0 = .0001;

      A aug_init = A(sigma0, 0, 0);

      Vector2d env_center, env_size, goal_center, goal_size;
      env_center << 0, 0;
      env_size << 50, 50;
      RegionCSpace<2> * c_space = new RegionCSpace<2> (new Box(env_center, env_size));

      goal_center << 23, 23;
      goal_size << 4, 4;
      Region<2> * goal_region = new Box(goal_center, goal_size);
      X x_init;
      x_init << 0, 0, 0;

      DubinsBeacon * dub_sys = new DubinsBeacon(x_init, aug_init, c_space, goal_region, nsigma);
      Vector2d beacon_center;
      beacon_center << 6, 4;
      dub_sys->beacons.push_back(new Circle(beacon_center, 3));
      beacon_center << -2, 4;
      dub_sys->beacons.push_back(new Circle(beacon_center, 3));

      BeliefTreeKD<3, P, A, 2> * rrbt = new BeliefTreeKD<3, P, A, 2> (dub_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);

      return rrbt;
    }
  case 'l':
    ;
    {
      if (ball_radius_constant == DEF_BALL_RADIUS_CONSTANT) {
        ball_radius_constant = DEF_DUB_BALL_RADIUS_CONSTANT;
      }

      if (max_ball_radius == DEF_MAX_BALL_RADIUS) {
        max_ball_radius = DEF_DUB_MAX_BALL_RADIUS;
      }

      // environment for generating the propagation plots for Labmda importance
      typedef DubinsBeacon::X X;
      typedef DubinsBeacon::StateCov StateCov;
      typedef DubinsBeacon::P P;
      typedef DubinsBeacon::A A;

      double sigma0 = .0001;

      A aug_init = A(sigma0, 0, 0);

      Vector2d env_center, env_size, goal_center, goal_size, obs_center, obs_size;
      env_center << 0, 0;
      env_size << 30, 25;
      RegionCSpace<2> * c_space = new RegionCSpace<2> (new Box(env_center, env_size));

      obs_center << 5, 4;
      obs_size << 6, 6;
      c_space->obstacles.push_back(new Box(obs_center, obs_size));
      obs_center << 5, -4;
      obs_size << 6, 6;
      c_space->obstacles.push_back(new Box(obs_center, obs_size));

      goal_center << 12, 0;
      goal_size << 6, 20;
      Region<2> * goal_region = new Box(goal_center, goal_size);

      X x_init;
      x_init << -12, 0, 0;

      DubinsBeacon * dub_sys = new DubinsBeacon(x_init, aug_init, c_space, goal_region, nsigma);
      Vector2d beacon_center;
      beacon_center << 2, 1;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 2, -1;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 8, 1;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 8, -1;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 2, 7;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 2, -7;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 8, 7;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));
      beacon_center << 8, -7;
      dub_sys->beacons.push_back(new Circle(beacon_center, 4));

//      dub_sys->K *= .2;

      BeliefTreeKD<3, P, A, 2> * rrbt = new BeliefTreeKD<3, P, A, 2> (dub_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);

      return rrbt;
    }

  case 'm':
    ;
    {

      if (ball_radius_constant == DEF_BALL_RADIUS_CONSTANT) {
        ball_radius_constant = DEF_DUB_BALL_RADIUS_CONSTANT;
      }

      if (max_ball_radius == DEF_MAX_BALL_RADIUS) {
        max_ball_radius = DEF_DUB_MAX_BALL_RADIUS;
      }

      // environment for generating the uncertain motion plot
      typedef DubinsBeacon::X X;
      typedef DubinsBeacon::StateCov StateCov;
      typedef DubinsBeacon::P P;
      typedef DubinsBeacon::A A;

      double sigma0 = .0001;

      A aug_init = A(sigma0, 0, 0);

      Vector2d env_center, env_size, goal_center, goal_size, obs_center, obs_size;
      env_center << 0, 0;
      env_size << 10, 5;
      RegionCSpace<2> * c_space = new RegionCSpace<2> (new Box(env_center, env_size));

      goal_center << 2, 0;
      goal_size << 2, 5;
      Region<2> * goal_region = new Box(goal_center, goal_size);

      X x_init;
      x_init << -4, -2, M_PI / 2;

      DubinsBeacon * dub_sys = new DubinsBeacon(x_init, aug_init, c_space, goal_region, nsigma);
      Vector2d beacon_center;
      beacon_center << 0, 2;
      dub_sys->beacons.push_back(new Circle(beacon_center, 10));
      beacon_center << 0, -2;
      dub_sys->beacons.push_back(new Circle(beacon_center, 10));

      dub_sys->K *= 0;

      dub_sys->Mcbase *= 2;

      dub_sys->Rc_beacon_base *= .0001;

      BeliefTreeKD<3, P, A, 2> * rrbt = new BeliefTreeKD<3, P, A, 2> (dub_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);

      return rrbt;
    }

  case 'z':
    ;
    {
      if (ball_radius_constant == DEF_BALL_RADIUS_CONSTANT) {
        ball_radius_constant = DEF_DUB_BALL_RADIUS_CONSTANT;
      }

      if (max_ball_radius == DEF_MAX_BALL_RADIUS) {
        max_ball_radius = DEF_DUB_MAX_BALL_RADIUS;
      }

      // environment for generating the uncertain measurement plots
      typedef DubinsBeacon::X X;
      typedef DubinsBeacon::StateCov StateCov;
      typedef DubinsBeacon::P P;
      typedef DubinsBeacon::A A;

      double sigma0 = .0001;

      A aug_init = A(sigma0, 0, 0);

      Vector2d env_center, env_size, goal_center, goal_size, obs_center, obs_size;
      env_center << 0, 0;
      env_size << 20, 10;
      RegionCSpace<2> * c_space = new RegionCSpace<2> (new Box(env_center, env_size));

      goal_center << 8, 0;
      goal_size << 2, 10;
      Region<2> * goal_region = new Box(goal_center, goal_size);

      X x_init;
      x_init << -7, 0, 0;

      DubinsBeacon * dub_sys = new DubinsBeacon(x_init, aug_init, c_space, goal_region, nsigma);
      Vector2d beacon_center;
      beacon_center << -3, 1;
      dub_sys->beacons.push_back(new Circle(beacon_center, 2));
      beacon_center << 3, -1;
      dub_sys->beacons.push_back(new Circle(beacon_center, 2));

      dub_sys->K *= 10;

      dub_sys->Mcbase *= 2;

      dub_sys->Rc_beacon_base *= .01;

      BeliefTreeKD<3, P, A, 2> * rrbt = new BeliefTreeKD<3, P, A, 2> (dub_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);

      return rrbt;
    }

  case 'u':
    ;
    {

      if (ball_radius_constant == DEF_BALL_RADIUS_CONSTANT) {
        ball_radius_constant = DEF_DUB_BALL_RADIUS_CONSTANT;
      }

      if (max_ball_radius == DEF_MAX_BALL_RADIUS) {
        max_ball_radius = DEF_DUB_MAX_BALL_RADIUS;
      }

      //urban canyon type environment
      typedef DubinsBeacon::X X;
      typedef DubinsBeacon::StateCov StateCov;
      typedef DubinsBeacon::P P;
      typedef DubinsBeacon::A A;

      double sigma0 = .0001;

      A aug_init = A(sigma0, 0, 0);

      Vector2d env_center, env_size, goal_center, goal_size, obs_center, obs_size;
      env_center << 14, 14;
      env_size << 34, 34;
      RegionCSpace<2> * c_space = new RegionCSpace<2> (new Box(env_center, env_size));

      goal_center << 14, 29.5;
      goal_size << 34, 3;
      Region<2> * goal_region = new Box(goal_center, goal_size);

      X x_init;
      x_init << 10, 0, M_PI / 4;

      double beacon_radius = 2;
      DubinsBeacon * dub_sys = new DubinsBeacon(x_init, aug_init, c_space, goal_region, nsigma);
      obs_center << 5.5, 6;
      obs_size << 6, 5;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 11, 12;
      obs_size << 8, 4;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 3, 18;
      obs_size << 6, 4;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 8, 17;
      obs_size << 4, 2;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 8.5, 24;
      obs_size << 3, 4;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 17.5, 24;
      obs_size << 9, 4;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 24.5, 19;
      obs_size << 3, 8;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);
      obs_center << 19.5, 10;
      obs_size << 4, 7;
      dub_sys->addSquareWithBeaconCorners(new Box(obs_center, obs_size), beacon_radius);

      dub_sys->Mcbase = .003 * Matrix2d::Identity();

      BeliefTreeKD<3, P, A, 2> * rrbt = new BeliefTreeKD<3, P, A, 2> (dub_sys, ball_radius_constant, max_ball_radius,
          epsilon, draw_mod, draw_animate, animate_usleep, RRBT_mode, search_order);

      return rrbt;
    }

  default:
    printf("unknown environment selection %c\n", sys_type);
  }
}

static void usage(const char *progname)
{
  char *basename = g_path_get_basename(progname);
  printf(
      "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "\n"
        "    -h, --help                shows this help text and exits\n"
        "    -d, --draw                publishes lcmgl messages to visualize the tree at each iteration, modulo optional arg (default %d)\n"
        "    -a, --animate             lcmgl animates propagate and steer for visualization, optional arg sets speed (default %d)\n"
        "    -n, --num_iter            set number of iterations (default %d)\n"
        "    -b, --ball_constant       constant to scale log(n)/sqrt(n) (default %f)\n"
        "    -r, --max_ball_radius     max_ball_radius (default %f)\n"
        "    -m, --epsilon             epsilon (default %f)\n"
        "    -e, --environment         environment\n"
        "                              g=gradient,p=pathological,h=homogeneous (default %c)\n"
        "    -o, --output              output iteration statistics to file, optional file name argument (default %s)\n"
        "    -p, --lqgmp               run lqg-mp algorithm instead of rrbt\n"
        "    -c, --chance              sigma bound for chance constraint (default %f)\n"
        "    -s, --seed                set random seed deterministically (otherwise set with clock) default 0\n"
        "    -w, --numsim              number of times to simulate the optimal path (default %d)\n"
        "    -q, --queue               queue ordering, ((u) uniform cost, d depth first, b breadth first)\n"
        "\n", basename, DEF_DRAW_MOD, DEF_ANIMATE_USLEEP, DEF_NUM_SAMPLES, DEF_BALL_RADIUS_CONSTANT,
      DEF_MAX_BALL_RADIUS, DEF_EPSILON, DEF_ENVIRONMENT, DEF_OUTPUT_FILENAME, DEF_NSIGMA, DEF_NUM_SIM_OPT_PATH);
  free(basename);
  exit(1);
}

int main(int argc, char **argv)
{
  //initialize default settings for algorithm
  double epsilon = DEF_EPSILON;
  double max_ball_radius = DEF_MAX_BALL_RADIUS;
  double ball_radius_constant = DEF_BALL_RADIUS_CONSTANT;
  bool RRBT_mode = true;
  search_t search_order = uniform_cost;
  bool random_seed = true;
  int seed = 0;
  int num_sim_opt_path = DEF_NUM_SIM_OPT_PATH;
  char env_type = DEF_ENVIRONMENT;
  double nsigma = DEF_NSIGMA;

  //initialize default global settings
  int num_iterations = DEF_NUM_SAMPLES;
  char temp_output_filename[100];
  bool specified_filename = false;
  bool output = false;
  char file_name[100] = DEF_OUTPUT_FILENAME;

  bool first_found = false;

  //initialize drawing settings
  bt_visualization_t draw_animate = bt_no_visualization;
  int animate_usleep = 0;
  int draw_mod = DEF_DRAW_MOD;

  char opt_string[20];

  const char *optstring = "hd::a::n:b:r:m:e:o::pc:s::fw:q:";
  struct option long_opts[] = { { "help", no_argument, 0, 'h' },
      { "draw", optional_argument, 0, 'd' },
      { "animate", optional_argument, 0, 'a' },
      { "num_iter", required_argument, 0, 'n' },
      { "ball_constant", required_argument, 0, 'b' },
      { "max_ball_radius", required_argument, 0, 'r' },
      { "epsilon", required_argument, 0, 'm' },
      { "environment", required_argument, 0, 'e' },
      { "output", optional_argument, 0, 'o' },
      { "lqgmp", no_argument, 0, 'p' },
      { "chance", required_argument, 0, 'c' },
      { "seed", required_argument, 0, 's' },
      { "first", no_argument, 0, 'f' },
      { "numsim", required_argument, 0, 'w' },
      { "queue", required_argument, 0, 'q' },
      { 0, 0, 0, 0 } };

  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'h':
      usage(argv[0]);

      break;
    case 'd':
      draw_animate = bt_draw;
      if (optarg != NULL)
        draw_mod = atoi(optarg);
      break;
    case 'a':
      draw_animate = bt_animate;
      if (optarg != NULL)
        animate_usleep = atoi(optarg);
      else
        animate_usleep = DEF_ANIMATE_USLEEP;
      break;
    case 'n':
      num_iterations = atoi(optarg);
      sprintf(opt_string, "_n%d", num_iterations);
      strcat(file_name, opt_string);
      break;
    case 'b':
      ball_radius_constant = atof(optarg);
      sprintf(opt_string, "_b%d", (int) ball_radius_constant);
      strcat(file_name, opt_string);
      break;
    case 'r':
      max_ball_radius = atof(optarg);
      sprintf(opt_string, "_r%d", (int) max_ball_radius);
      strcat(file_name, opt_string);
      break;
    case 'm':
      epsilon = atof(optarg);
      sprintf(opt_string, "_r%d", (int) epsilon);
      strcat(file_name, opt_string);
      break;
    case 'e':
      env_type = optarg[0];
      sprintf(opt_string, "_e%c", env_type);
      strcat(file_name, opt_string);
      break;
    case 'o':
      if (optarg != NULL) {
        strcpy(temp_output_filename, optarg);
        specified_filename = true;
      }
      output = true;
      break;
    case 'p':
      RRBT_mode = false;
      strcat(file_name, "_p");
      break;
    case 'c':
      nsigma = atof(optarg);
      sprintf(opt_string, "_c%d", (int) nsigma);
      strcat(file_name, opt_string);
      break;
    case 's':
      if (optarg != NULL) {
        seed = atoi(optarg);
      }

      random_seed = false;
      sprintf(opt_string, "_s%d", seed);
      strcat(file_name, opt_string);
      break;

    case 'f':
      sprintf(opt_string, "_f");
      strcat(file_name, opt_string);
      first_found = true;
      break;

    case 'w':
      num_sim_opt_path = atoi(optarg);
      sprintf(opt_string, "_w%d", (int) num_sim_opt_path);
      strcat(file_name, opt_string);
      break;
    case 'q':
      if (optarg[0] == 'u')
        search_order = uniform_cost;
      else if (optarg[0] == 'd')
        search_order = depth_first;
      else if (optarg[0] == 'b')
        search_order = breadth_first;
      else
        usage(argv[0]);
      sprintf(opt_string, "_q%c", optarg[0]);
      strcat(file_name, opt_string);
      break;
    default:
      usage(argv[0]);
      break;
    }
  }

  if (specified_filename) {
    strcpy(file_name, temp_output_filename);
  }

  if (optind < argc - 1) {
    usage(argv[0]);
  }

  int ii = 0;

  if (random_seed) {
    srand(time(NULL));
  }
  else {
    srand(seed);
  }

  //setup output stat arrays
  std::vector<double> optimal_cost(num_iterations, 0);
  std::vector<int> num_mu_nodes(num_iterations, 0);
  std::vector<int> num_belief_nodes(num_iterations, 0);
  std::vector<uint64_t> time(num_iterations, 0);

  //timing variables
  uint64_t run_time = 0;
  uint64_t start_time;

  FILE * stats_file = NULL;
  if (output) {
    stats_file = fopen(file_name, "wt");
    printf("output filename: %s\n", file_name);
    //fprintf(stats_file,"num_samples, num_mean_nodes, num_belief_nodes, running_time, min_cost_at_goal\n");
  }

  int64_t next_cout_time = bot_timestamp_now();

  BeliefTreeInterface * rrbt = getBeliefTree(env_type, ball_radius_constant, max_ball_radius, epsilon, draw_mod,
      draw_animate, animate_usleep, RRBT_mode, nsigma, search_order);

  double its_between = 0;
  double total_near = 0;
  double num_near;

  while (ii < num_iterations) {
    start_time = bot_timestamp_now();
    rrbt->iterateRRBT();
    run_time += bot_timestamp_now() - start_time;

    time[ii] = run_time;
    optimal_cost[ii] = rrbt->getMinCostAtGoal();
    num_mu_nodes[ii] = rrbt->getNumStateVertices();
    num_belief_nodes[ii] = rrbt->getNumBeliefNodes();

    if (optimal_cost[ii] > 0 && first_found) {
      break;
    }

    total_near += rrbt->getNumNearLast();
    its_between += 1;

    if (output) {
      fprintf(stats_file, "%d, %d, %d, %u, %f\n", ii, num_mu_nodes[ii], num_belief_nodes[ii], (unsigned int) time[ii],
          optimal_cost[ii]);
    }

    if (start_time > next_cout_time) {
      num_near = total_near / its_between;
      total_near = 0;
      its_between = 0;
      cout << ((double) time[ii]) * 1e-6 << "s, "  << ii << " iterations, " << num_mu_nodes[ii] << " state nodes, " << num_belief_nodes[ii]
          << " belief nodes, " << optimal_cost[ii] << " cost, " << rrbt->getBallRadius() << " ball radius, "
          << num_near << " average near vertices " << endl;
      next_cout_time += 1e6;
    }

    ii++;
  }

  cout << ((double) time[ii - 1]) * 1e-6 << "s, "  << ii << " iterations, " << num_mu_nodes[ii - 1] << " state nodes, " << num_belief_nodes[ii
      - 1] << " belief nodes, " << optimal_cost[ii - 1] << " cost\n";

  rrbt->lcmgl_draw();

  double frac_success = rrbt->simOptPath(num_sim_opt_path);

  cout << frac_success << "\% success executing path\n";

  // put the fractional success in the cost column
  if (output) {
    ii -= 1;
    fprintf(stats_file, "%d, %d, %d, %u, %f\n", ii, num_mu_nodes[ii], num_belief_nodes[ii], (unsigned int) time[ii],
        frac_success);
    fclose(stats_file);

  }

  delete rrbt;

  return 0;
}

