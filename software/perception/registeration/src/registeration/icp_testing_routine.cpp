// Code for registration of 10 clouds 
//in all of their possible combinations.

#include "icp_testing_routine.hpp"

RoutineConfig::RoutineConfig(){
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  initFilename.append(homedir);
  initFilename.append("/logs/multisenselog__2015-11-16/initialization/initTransf.txt");

  num_clouds = 10;
}

RegistrationRoutine::RegistrationRoutine():
  cfg_(RoutineConfig()){
  init();
}

void RegistrationRoutine::init(){
  cols = 0;
  for (int i = cfg_.num_clouds-1; i > 0; i--)
  {
    cols = cols + i;
  }
}


RegistrationRoutine::~RegistrationRoutine()
{
}

void RegistrationRoutine::doRoutine(Eigen::MatrixXf &transf_matrix)
{
  //-------------------------
  // To store...
  transf_matrix = Eigen::MatrixXf::Zero(3, cols);
  //-------------------------

  cout << "Start computing transformations:" << endl;

  int transf_index = 0;
  string cloud_name_A, cloud_name_B;

  cout << "initFilename: " << cfg_.initFilename <<endl;

  for (int i = 0; i < cfg_.num_clouds-1; i++)
  {
    // Load reference cloud from file
    cloud_name_A.clear();
    cloud_name_A.append(cfg_.homedir);
    cloud_name_A.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_0");
    cloud_name_A.append(to_string(i));
    cloud_name_A.append(".vtk");

    DP ref = DP::load(cloud_name_A);

    for (int j = 1+i; j < cfg_.num_clouds; j++)
    {
      cout << transf_index+1 << endl;

      // Load inital transformations from file
      cfg_.initTrans = readLineFromFile(cfg_.initFilename, transf_index);
      cout << "initTrans: " << cfg_.initTrans <<endl;
      if (cfg_.initTrans.empty())
        cfg_.initTrans.append("0,0,0");
      else
        transf_index++;

      //cout << "initTrans_: " << initTrans_ << endl;

      // Load input cloud from file
      cloud_name_B.clear();
      cloud_name_B.append(cfg_.homedir);
      cloud_name_B.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_0");
      cloud_name_B.append(to_string(j));
      cloud_name_B.append(".vtk");

      //cout << "cloud_name_A: " << cloud_name_A <<endl;
      //cout << "cloud_name_B: " << cloud_name_B <<endl;

      ref = DP::load(cloud_name_A);
      DP data = DP::load(cloud_name_B);
      
      //=================================
      // TRANSFORM 3D CLOUD
      //=================================

      cloudDim = ref.getEuclideanDim();

      PM::TransformationParameters T = PM::TransformationParameters::Identity(4,4);
      getICPTransform(data, ref, T);

      cout << "3D Transformation:" << endl << T << endl; 

      transf_matrix(0, transf_index-1) = T(0,cloudDim);
      transf_matrix(1, transf_index-1) = T(1,cloudDim);
      transf_matrix(2, transf_index-1) = atan2 (T(1,0),T(0,0)) * 180 / M_PI;
    }
  }

  cout << "Completed!" << endl;
}

void RegistrationRoutine::getICPTransform(DP &cloud_in, DP &cloud_ref, PM::TransformationParameters &T)
{
  // Create the default ICP algorithm
  PM::ICP icp;
  
  if (!(cloudDim == 2 || cloudDim == 3)) 
  {
    cerr << "Invalid input point clouds dimension." << endl;
    exit(1);
  }

  if (cloudDim == 2)
  {
    // ICP chain configuration: check if prefiltering required
    if (configFile2D.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
    else
    {
      // load YAML config
      ifstream ifs(configFile2D.c_str());
      if (!ifs.good())
      {
        cerr << "Cannot open config file " << configFile2D << endl; exit(1);
      }
      icp.loadFromYaml(ifs);
      //cout << "Loaded pre-filtering chain from yaml..." << endl;
    }
  }
  else
  {
    // ICP chain configuration: check if prefiltering required
    if (configFile3D.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
    else
    {
      // load YAML config
      ifstream ifs(configFile3D.c_str());
      if (!ifs.good())
      {
        cerr << "Cannot open config file " << configFile3D << endl; exit(1);
      }
      icp.loadFromYaml(ifs);
      //cout << "Loaded pre-filtering chain from yaml..." << endl;
    }
  }

  // Apply rigid transformation (just a "visually good" approximation of the transformation 
  // between ref and input clouds) to escape local minima
  PM::TransformationParameters initTransfo = parseTransformationDeg(cfg_.initTrans, cloudDim);

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(initTransfo)) {
    cerr << endl
       << "Initial transformation is not rigid, identity will be used."
       << endl;
    initTransfo = PM::TransformationParameters::Identity(
          cloudDim+1,cloudDim+1);
  }

  const DP initializedData = rigidTrans->compute(cloud_in, initTransfo);

  // Compute the transformation to express data in ref
  T = icp(cloud_in, cloud_ref, initTransfo);
  // Uncomment this and comment previous line to
  // compute the transformation to express initialized data in ref 
  //T = icp(initializedData, cloud_ref);
  cout << "Match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

  // Transform data to express it in ref
  DP data_out(cloud_in);
  icp.transformations.apply(data_out, T);

  //cout << "Final 3D transformation:" << endl << T << endl;

  //======================== Errors............... ==========================
  //computeCloudsDistance (icp, cloud_ref, data_out);
  //=========================================================================
}

// Make sure that the command arguments make sense
int RegistrationRoutine::validateArgs(const int argc, const char *argv[])
{
  if (argc < 3)
  {
    cerr << "Needs config files for ICP chain definition.\n";
    cerr << "Usage:";
    usage(argv);
    return -1;
  }
 
  const int endOpt(argc);

  for (int i = 1; i < endOpt; i += 2)
  {
    const string opt(argv[i]);
    if (i + 1 > endOpt)
    {
      cerr << "Missing value for option " << opt << ", usage:"; usage(argv); exit(1);
    }
    if (opt == "--config2D") {
      configFile2D.append(getenv("DRC_BASE"));
      configFile2D.append("/software/perception/registeration/filters_config/");
      configFile2D.append(argv[i+1]);
    }
    else if (opt == "--config3D") {
      configFile3D.append(getenv("DRC_BASE"));
      configFile3D.append("/main-distro/software/perception/registeration/filters_config/");
      configFile3D.append(argv[i+1]);
    }
    else
    {
      cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
    }
  }
  return 0;
}

// Dump command-line help
void RegistrationRoutine::usage(const char *argv[])
{
  //TODO: add new options --isTransfoSaved, --initTranslation, --initRotation
  cerr << endl << endl;
  cerr << "* To run ICP:" << endl;
  cerr << "  " << argv[0] << " [OPTIONS]" << endl;
  cerr << endl;
  cerr << "OPTIONS must be a combination of:" << endl;
  cerr << "--config2D YAML_2DCLOUD_CONFIG_FILE  Load the config from a YAML file located in filters_config (default: default parameters)" << endl;
  cerr << "--config3D YAML_3DCLOUD_CONFIG_FILE  Load the config from a YAML file located in filters_config (default: default parameters)" << endl;
  cerr << endl;
}

