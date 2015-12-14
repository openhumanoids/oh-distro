#include "icp_utils.h"

/* Compute some metrics given registration between a reference and an input cloud 
(Haussdorff distance, Haussdorff quantile distance and Robust mean distance in meters). */

void computeCloudsDistance (PM::ICP &icp, DP &cloud_ref, DP &data_out)
{
  cout << endl << "------------------" << endl;
	
  // START demo 1
  // Test for retrieving Haussdorff distance (with outliers). We generate new matching module 
  // specifically for this purpose. 
  //
  // INPUTS:
  // ref: point cloud used as reference
  // data_out: aligned point cloud (using the transformation outputted by icp)
  // icp: icp object used to aligned the point clouds

	
  // Structure to hold future match results
  PM::Matches matches;

  Parametrizable::Parameters params;
  params["knn"] =  toParam(1); // for Hausdorff distance, we only need the first closest point
  params["epsilon"] =  toParam(0);

  PM::Matcher* matcherHausdorff = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);
	
  // The Hausdorff distance is the greatest of all the distances from a point in one set to the closest point in the other set.
  // max. distance from reading to reference
  matcherHausdorff->init(cloud_ref);
  matches = matcherHausdorff->findClosests(data_out);
  float maxDist1 = matches.getDistsQuantile(1.0);
  float maxDistRobust1 = matches.getDistsQuantile(0.85);

  // max. distance from reference to reading
  matcherHausdorff->init(data_out);
  matches = matcherHausdorff->findClosests(cloud_ref);
  float maxDist2 = matches.getDistsQuantile(1.0);
  float maxDistRobust2 = matches.getDistsQuantile(0.85);

  float haussdorffDist = std::max(maxDist1, maxDist2);
  float haussdorffQuantileDist = std::max(maxDistRobust1, maxDistRobust2);

  cout << "Haussdorff distance: " << std::sqrt(haussdorffDist) << " m" << endl;
  cout << "Haussdorff quantile distance: " << std::sqrt(haussdorffQuantileDist) <<  " m" << endl;

  // START demo 2
  // Test for retrieving paired point mean distance without outliers. We reuse the same module used for 
  // the icp object.
  //
  // INPUTS:
  // ref: point cloud used as reference
  // data_out: aligned point cloud (using the transformation outputted by icp)
  // icp: icp object used to aligned the point clouds
	
  // initiate the matching with unfiltered point cloud
  icp.matcher->init(cloud_ref);

  // extract closest points
  matches = icp.matcher->findClosests(data_out);

  // weight paired points
  const PM::OutlierWeights outlierWeights = icp.outlierFilters.compute(data_out, cloud_ref, matches);
	
  // generate tuples of matched points and remove pairs with zero weight
  const PM::ErrorMinimizer::ErrorElements matchedPoints = icp.errorMinimizer->getMatchedPoints( data_out, cloud_ref, matches, outlierWeights);

  // extract relevant information for convenience
  const int dim = matchedPoints.reading.getEuclideanDim();
  const int nbMatchedPoints = matchedPoints.reading.getNbPoints(); 
  const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim);
  const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);
	
  // compute mean distance
  const PM::Matrix dist = (matchedRead - matchedRef).colwise().norm(); // replace that by squaredNorm() to save computation time
  const float meanDist = dist.sum()/nbMatchedPoints;
  cout << "Robust mean distance: " << meanDist << " m" << endl;

  // END demo

  cout << "------------------" << endl << endl;
}

/* Get the line whose index is given as argument from file. */

string readLineFromFile(string& filename, int line_number)
{
  string lines, line;
  int index = 0;

  ifstream trasfFile (filename);
  if (trasfFile.is_open())
  {
    while ( getline (trasfFile, lines) )
    {
      if (index == line_number)
      {
      	getline (trasfFile, line);
      }
      index ++;
    }
    trasfFile.close();
  }
  else cout << "Unable to open init tranform file."; 

  return line;
}

/* Get a transformation matrix (of the type defined in the libpointmatcher library)
given a string containing info about a translation on the plane x-y and a rotation 
about the vertical axis z, i.e. [x,y,theta] (meters,meters,radians). */

PM::TransformationParameters parseTransformation(string& transform, const int cloudDimension) 
{
  PM::TransformationParameters parsedTrans;
  parsedTrans = PM::TransformationParameters::Identity(
        cloudDimension+1,cloudDimension+1);

  transform.erase(std::remove(transform.begin(), transform.end(), '['),
            transform.end());
  transform.erase(std::remove(transform.begin(), transform.end(), ']'),
            transform.end());
  std::replace( transform.begin(), transform.end(), ',', ' ');
  std::replace( transform.begin(), transform.end(), ';', ' ');

  float transValues[3] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 3; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "An error occured while trying to parse the initial "
         << "transformation." << endl
         << "No initial transformation will be used" << endl;
      return parsedTrans;
    }
  }

  for( int i = 0; i < 3; i++) {
    if (i == 2)
    {
      parsedTrans(i-2,i-2) = cos ( transValues[i] );
      parsedTrans(i-2,i-1) = - sin ( transValues[i] );
      parsedTrans(i-1,i-2) = sin ( transValues[i] );
      parsedTrans(i-1,i-1) = cos ( transValues[i] );
    }
    else
    {
      parsedTrans(i,cloudDimension) = transValues[i];
    }
  }

  //cout << "Parsed initial transformation:" << endl << parsedTrans << endl;

  return parsedTrans;
}

/* Get a transformation matrix (of the type defined in the libpointmatcher library)
given a string containing info about a translation on the plane x-y and a rotation 
about the vertical axis z, i.e. [x,y,theta] (meters,meters,degrees). */

PM::TransformationParameters parseTransformationDeg(string& transform, const int cloudDimension) 
{
  PM::TransformationParameters parsedTrans;
  parsedTrans = PM::TransformationParameters::Identity(
        cloudDimension+1,cloudDimension+1);

  transform.erase(std::remove(transform.begin(), transform.end(), '['),
            transform.end());
  transform.erase(std::remove(transform.begin(), transform.end(), ']'),
            transform.end());
  std::replace( transform.begin(), transform.end(), ',', ' ');
  std::replace( transform.begin(), transform.end(), ';', ' ');

  float transValues[3] = {0};
  stringstream transStringStream(transform);
  for( int i = 0; i < 3; i++) {
    if(!(transStringStream >> transValues[i])) {
      cerr << "An error occured while trying to parse the initial "
         << "transformation." << endl
         << "No initial transformation will be used" << endl;
      return parsedTrans;
    }
  }

  for( int i = 0; i < 3; i++) {
    if (i == 2)
    {
      parsedTrans(i-2,i-2) = cos ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-2,i-1) = - sin ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-1,i-2) = sin ( transValues[i] * M_PI / 180.0 );
      parsedTrans(i-1,i-1) = cos ( transValues[i] * M_PI / 180.0 );
    }
    else
    {
      parsedTrans(i,cloudDimension) = transValues[i];
    }
  }

  //cout << "Parsed initial transformation:" << endl << parsedTrans << endl;

  return parsedTrans;
}

void fromDataPointsToPCL(DP &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
  cloud_out.points.resize(cloud_in.getNbPoints());
  for (int i = 0; i < cloud_in.getNbPoints(); i++) {
    cloud_out.points[i].x = (cloud_in.features.col(i))[0];
    cloud_out.points[i].y = (cloud_in.features.col(i))[1];
    cloud_out.points[i].z = (cloud_in.features.col(i))[2];
    //cout << "i=" << i << " " << cloud_out.points[i].x << " " << cloud_out.points[i].y << " " << cloud_out.points[i].z << endl;
  }  
  cloud_out.width = cloud_out.points.size();
  cloud_out.height = 1;
}

void writeTransformToFile(Eigen::MatrixXf &transformations, string out_file, int num_clouds)
{
  std::vector<string> v;
  for (int i = 0; i < num_clouds-1; i++)
  {
    for (int j = 1+i; j < num_clouds; j++)
    {
      string str;
      str.append(to_string(i));
      str.append(to_string(j));
      v.push_back(str);
    }
  }

  ofstream file (out_file);

  if (file.is_open())
  {
    //file << "x y theta\n";
    for (int i = 0; i < v.size(); i++)
    {
      file << v[i] << " " << transformations(0,i) << " " << transformations(1,i) 
      << " " << transformations(2,i) << endl;
    }
    file.close();
  }
  else cout << "Unable to open file";
  cout << "Written file: " << out_file << endl;
}