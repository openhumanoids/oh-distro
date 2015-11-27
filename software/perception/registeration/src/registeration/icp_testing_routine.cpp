// cddrc
// drc-icp-testing-routine

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include "boost/filesystem.hpp"

#include "cloud_accumulate.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "icp_utils.h"

using namespace std;

const char *homedir;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

string configFile2D_, configFile3D_;
string initTrans_;

int icp_testing_routine(int argc, const char *argv[]);

void getICPTransform(DP &cloud_in, DP &cloud_ref);
void applyRigidTransform(DP &pointCloud);
void publishCloud(int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_topub);
void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);

int validateArgs(const int argc, const char *argv[]);
void usage(const char *argv[]);

/**
  * Code for ICP testing...
  */

int main(int argc, char const *argv[])
{
  return icp_testing_routine(argc, argv);
}

int icp_testing_routine(int argc, const char *argv[])
{
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  string filename;
  filename.append(homedir);
  filename.append("/logs/multisenselog__2015-11-16/initialization/initTransf.txt");

  const int ret = validateArgs(argc, argv);

  if (ret == -1)
    return ret;

  int tot_clouds = 10;

  string cloud_name_A, cloud_name_B;
  DP ref, data;
  int transf_index = 0;

  cout << "Start computing transformations:" << endl;

  for (int i = 0; i < tot_clouds-1; i++)
  {
    for (int j = 1+i; j < tot_clouds; j++)
    {
      cout << transf_index+1 << endl;

      // Load inital transformations from file
      initTrans_ = readLineFromFile(filename, transf_index);
      if (initTrans_.empty())
        initTrans_.append("0,0,0");
      else
        transf_index++;

      //cout << "initTrans_: " << initTrans_ << endl;

      // Load clouds from file
      cloud_name_A.clear();
      cloud_name_A.append(homedir);
      cloud_name_A.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_0");
      cloud_name_A.append(to_string(i));
      cloud_name_A.append(".vtk");

      cloud_name_B.clear();
      cloud_name_B.append(homedir);
      cloud_name_B.append("/logs/multisenselog__2015-11-16/pointclouds/multisense_0");
      cloud_name_B.append(to_string(j));
      cloud_name_B.append(".vtk");

      ref = DP::load(cloud_name_A);
      data = DP::load(cloud_name_B);
      
      //=================================
      // TRANSFORM 3D CLOUD
      //=================================

      getICPTransform(data, ref);
    }
  }

  cout << "Completed!" << endl;

  return 0;
}

//========================================================
// FUNCTIONS
//========================================================

// Make sure that the command arguments make sense
int validateArgs(const int argc, const char *argv[])
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
      configFile2D_.append(homedir);
      configFile2D_.append("/main-distro/software/perception/registeration/filters_config/");
      configFile2D_.append(argv[i+1]);
    }
    else if (opt == "--config3D") {
      configFile3D_.append(homedir);
      configFile3D_.append("/main-distro/software/perception/registeration/filters_config/");
      configFile3D_.append(argv[i+1]);
    }
    else
    {
      cerr << "Unknown option " << opt << ", usage:"; usage(argv); exit(1);
    }
  }
  return 0;
}

// Dump command-line help
void usage(const char *argv[])
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


void getICPTransform(DP &cloud_in, DP &cloud_ref)
{
  // Create the default ICP algorithm
  PM::ICP icp;

  int cloudDimension = cloud_ref.getEuclideanDim();
  
  if (!(cloudDimension == 2 || cloudDimension == 3)) 
  {
    cerr << "Invalid input point clouds dimension." << endl;
    exit(1);
  }

  if (cloudDimension == 2)
  {
    // ICP chain configuration: check if prefiltering required
    if (configFile2D_.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
    else
    {
      // load YAML config
      ifstream ifs(configFile2D_.c_str());
      if (!ifs.good())
      {
        cerr << "Cannot open config file " << configFile2D_ << endl; exit(1);
      }
      icp.loadFromYaml(ifs);
      //cout << "Loaded pre-filtering chain from yaml..." << endl;
    }
  }
  else
  {
    // ICP chain configuration: check if prefiltering required
    if (configFile3D_.empty())
    {
      // See the implementation of setDefault() to create a custom ICP algorithm
      icp.setDefault();
    }
    else
    {
      // load YAML config
      ifstream ifs(configFile3D_.c_str());
      if (!ifs.good())
      {
        cerr << "Cannot open config file " << configFile3D_ << endl; exit(1);
      }
      icp.loadFromYaml(ifs);
      //cout << "Loaded pre-filtering chain from yaml..." << endl;
    }
  }

  // Apply rigid transformation (just a "visually good" approximation of the transformation 
  // between ref and input clouds) to escape local minima
  PM::TransformationParameters initTransfo = parseTransformation(initTrans_, cloudDimension);

  PM::Transformation* rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(initTransfo)) {
    cerr << endl
       << "Initial transformation is not rigid, identity will be used."
       << endl;
    initTransfo = PM::TransformationParameters::Identity(
          cloudDimension+1,cloudDimension+1);
  }

  const DP initializedData = rigidTrans->compute(cloud_in, initTransfo);

  // Compute the transformation to express data in ref
  PM::TransformationParameters T = icp(initializedData, cloud_ref);
  //cout << "Match ratio: " << icp.errorMinimizer->getWeightedPointUsedRatio() << endl;

  // Transform data to express it in ref
  DP data_out(initializedData);
  icp.transformations.apply(data_out, T);

  //cout << "Final 3D transformation:" << endl << T << endl;

  //======================== Errors............... ==========================
  //computeCloudsDistance (icp, cloud_ref, data_out);
  //=========================================================================
}