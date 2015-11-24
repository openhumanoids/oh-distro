#include "icp_utils.h"

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
