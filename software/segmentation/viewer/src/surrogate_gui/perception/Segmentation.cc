/*
 * Segmentation.cpp
 *
 *  Created on: Jan 19, 2012
 *      Author: mfleder
 */

#include "Segmentation.h"
#include "PclSurrogateUtils.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>  //for computePointNormal
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/impl/centroid.hpp> //for computing centroid
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>

#include <bot_core/rotations.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

namespace surrogate_gui
{

  Matrix3f ypr2rot(Vector3f ypr){
    double rpy[]={ypr[2],ypr[1],ypr[0]};
    double q[4];
    double mat[9];
    bot_roll_pitch_yaw_to_quat(rpy,q);
    int rc=bot_quat_to_matrix(q,mat);
    Matrix3d mat2(mat);
    for(int j=0;j<3;j++){
      for(int i=0;i<3;i++){
	mat2(j,i) = mat[j*3+i];
      }
    }
    return mat2.cast<float>();
  }
  
  Vector3f rot2ypr(Matrix3f mat){
    Matrix3d mat2 = mat.transpose().cast<double>(); //convert from col-major to row-major
    double* mat3 = mat2.data();
    double q[4];
    double rpy[3];
    int rc=bot_matrix_to_quat(mat3,q);
    bot_quat_to_roll_pitch_yaw(q,rpy);
    return Vector3f(rpy[2],rpy[1],rpy[0]);
  }
  
	vector<PointIndices::Ptr> Segmentation::segment(const PointCloud<PointXYZRGB>::ConstPtr cloud,
							boost::shared_ptr<set<int> >  subcloudIndices)
	{
	  //return getPlanarComponents(cloud, subcloudIndices);
	  
	  vector<PointIndices::Ptr> planeComponents = getRansacSegments(cloud,
									PclSurrogateUtils::toPclIndices(subcloudIndices),
									3, SACMODEL_PLANE);
	  
	  cout << endl << "***found " << planeComponents.size() << " plane components***" << endl;;

	  vector<PointIndices::Ptr> segments;
	  for (uint i = 0; i < planeComponents.size(); i++)
	    {
	      vector<PointIndices::Ptr> nextEuclidSegments = getEuclideanClusters(cloud, planeComponents[i]);

			//circle components
			/*for (uint n = 0; n < nextEuclidSegments.size(); n++)
			{
				vector<PointIndices::Ptr> nextCircleComponents = getRansacSegments(cloud, nextEuclidSegments[n],
																			   1, SACMODEL_CIRCLE2D);

				segments.insert(segments.end(), nextCircleComponents.begin(), nextCircleComponents.end());
			}*/

			//cout << endl << "plane component split into " << nextEuclidSegments.size() << " euclids" << endl;
			segments.insert(segments.end(), nextEuclidSegments.begin(), nextEuclidSegments.end());
		}


		cout << endl << "returning " << segments.size() << " segments" << endl;
		return segments;
	}

	/**@returns a vector of the largest segments having the given shape
	 * @param cloud we search a subset of this cloud
	 * @param subcloudIndices subset of cloud to search for shapeToFind shapes
	 * @param maxNumSegments maximum number of segments to return
	 * @param shapeToFind shape we are looking for.  eg pcl::SACMODEL_PLANE*/
	vector<PointIndices::Ptr> Segmentation::getRansacSegments(const PointCloud<PointXYZRGB>::ConstPtr cloud,
															  PointIndices::Ptr subCloudIndices,
															  uint maxNumSegments, pcl::SacModel shapeToFind)
	{
		//====copy data
		PointIndices::Ptr subcloudCopy = PclSurrogateUtils::copyIndices(subCloudIndices);

		//create the segmentation object
		SACSegmentation<PointXYZRGB> seg;
		seg.setOptimizeCoefficients(true); //optional
		seg.setModelType(shapeToFind);
		seg.setMethodType(SAC_RANSAC);
		seg.setDistanceThreshold(0.01);

		//filtering object
		vector<PointIndices::Ptr> segmentsFound;  //planar/circle components we will return

		//extract the top 5 planar segments, or stop if no good plane fits left
		uint min_plane_size = (int) (0.2 * subcloudCopy->indices.size());
		if (min_plane_size > 500 || min_plane_size < 300)
			min_plane_size = 500;

		for (uint i = 0;
				i < maxNumSegments
				&& subcloudCopy->indices.size() > min_plane_size;
			 i++)
		{

			//set input
			seg.setInputCloud(cloud);
			seg.setIndices(subcloudCopy);

			//segment
			ModelCoefficients::Ptr coefficients(new ModelCoefficients);
			PointIndices::Ptr nextSegmentIndices (new PointIndices);
			seg.segment(*nextSegmentIndices, *coefficients);

			//good shape-fitting component?  todo: check the fit
			if (nextSegmentIndices->indices.size() > min_plane_size)
			{
				/*cout << endl << "next model size: " << planeIndices->indices.size() << " points with "
						<< coefficients->values.size() << " coefficients" << endl
						<< "R^2 = " << getPlaneFitStatistics(coefficients, cloud, planeIndices) << endl;
				*/
				segmentsFound.push_back(nextSegmentIndices); //found a shape-fitting component
			}
			else
				break; //no big segments left

			//=====remove this shape's (plane's) indices from pclSubCloudIndices
			//remainingIndices = pclSubCloudIndices - planeIndices
			boost::shared_ptr<set<int> > remainingIndices (new set<int>(subcloudCopy->indices.begin(),
																		subcloudCopy->indices.end()));
			for(uint i = 0; i < nextSegmentIndices->indices.size(); i++)
			{
				remainingIndices->erase(nextSegmentIndices->indices[i]);
			}

			subcloudCopy = PclSurrogateUtils::toPclIndices(remainingIndices);
		}
		return segmentsFound;
	}


	/**@returns a clustering of indicesToCLuster.  indices are with respect to cloud*/
	vector<PointIndices::Ptr> Segmentation::getEuclideanClusters(const PointCloud<PointXYZRGB>::ConstPtr cloud,
																 PointIndices::Ptr indicesToCluster)
	{
		if (cloud == PointCloud<PointXYZRGB>::Ptr())
			throw SurrogateException("Null cloud passed to getEuclideanClusters");
		if (indicesToCluster->indices.size() == 0)
			throw SurrogateException("empty indices passed to getEuclideanClusters");

		//KdTree requires indices passed in as shared_ptr<vector<int>>
		//but PointIndices::Ptr is of type shared_pt<PointIndices>
		//so we need to copy the vector<int> indices and create an IndicesConstPtr
		boost::shared_ptr<vector<int> > constIndicesToCluster(new vector<int>(indicesToCluster->indices));
		if (constIndicesToCluster->size() == 0)
			throw SurrogateException("Empty constIndicesToCluster");

		//------
/*		cout << "\n PCL 1.5.1 throws seg fault when trying to euclidean cluster.  skipping euclidean clusterin"
				<< endl;
		vector<PointIndices::Ptr> unmodifiedReturn;
		unmodifiedReturn.push_back(indicesToCluster);
		return unmodifiedReturn;*/
		//----

		//KdTree object for the search method of extraction
		search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
		tree->setInputCloud(cloud, constIndicesToCluster);

		//euclidean cluster object
		EuclideanClusterExtraction<PointXYZRGB> euclid;
		euclid.setClusterTolerance(0.02); //2cm
		euclid.setMinClusterSize(min(100, (int) (indicesToCluster->indices.size()/2.0)));
		euclid.setMaxClusterSize(indicesToCluster->indices.size());

		/*
		cout << "\n minClusterSize = " << euclid.getMinClusterSize() << endl;
		cout << "\n maxClusterSize = " << euclid.getMaxClusterSize() << endl;
		cout << "\n Cluster Tolerance = " << euclid.getClusterTolerance() << endl;
		*/

		euclid.setSearchMethod(tree);

		//set input
		euclid.setInputCloud(cloud);
		euclid.setIndices(indicesToCluster);

		//extract
		vector<PointIndices> cluster_indices;
		//cout << "\n about to run exract" << endl;
		euclid.extract(cluster_indices);  //pcl 1.5.1 throwing an error here

		//cout << endl << "=======got " << cluster_indices.size() << " clusters" << endl;

		//=======convert to pointer
		vector<PointIndices::Ptr> clustersAsPtrs;
		clustersAsPtrs.reserve(cluster_indices.size());

		for (uint i = 0; i < cluster_indices.size(); i++)
		{
			PointIndices::Ptr nextCluster(new PointIndices); //space for copy
			nextCluster->indices = cluster_indices[i].indices; //copy
			clustersAsPtrs.push_back(nextCluster);
		}

		return clustersAsPtrs;
	}


	/**http://en.wikipedia.org/wiki/Coefficient_of_determination
	 * @return R^2 value
	 *
	 * http://docs.pointclouds.org/trunk/group__sample__consensus.html#gad3677a8e6b185643a2b9ae981e831c14
	 * */
	double Segmentation::getPlaneFitStatistics(ModelCoefficients::Ptr planeCoeffs,
						   const PointCloud<PointXYZRGB>::ConstPtr cloud,
						   PointIndices::Ptr planeIndices)
	{
		//===========================================
		//====================================defensive checks
		if (planeCoeffs->values.size() != 4)
			throw SurrogateException("Excpecting 4 plane coefficients");
		if (planeIndices->indices.size() < 3)
			throw SurrogateException("Can't have a plane w/ < 3 points");

		//===========confirm the plane fit
		Eigen::Vector4f planeParameters;
		float curvature;

		pcl::computePointNormal(*cloud, planeIndices->indices, planeParameters, curvature);

		//cout << "==============recomputed plane params" << endl;
		for (int i = 0; i < planeParameters.size(); i++)
		{
			if (abs(planeCoeffs->values[i] - planeParameters[i]) > 0.1)
				cout << "\n *****expected " << planeCoeffs->values[i]
				 << " got " << planeParameters[i]
				 << " | difference = " << (planeCoeffs->values[i] - planeParameters[i])
				 << endl;
		}

		//================================
		//===============now compute stats

		//determine centroid
		Eigen::Vector4f centroid4f;
		compute3DCentroid(*cloud, planeIndices->indices, centroid4f);
		PointXYZ centroid(centroid4f[0], centroid4f[1], centroid4f[2]);

		//---get sum of squared errors and total sum of squares
		double sumSquaredErrors = 0;
		double sumAbsErrors = 0;
		double ssTot = 0;
		double maxError = -1;
		double minError = 999999999;
		double maxDist = 0;
		double minDist = 1000;
		for(vector<int>::iterator indIter = planeIndices->indices.begin();
			indIter != planeIndices->indices.end();
			indIter++)
		{
			const PointXYZRGB &nextPoint = cloud->points[*indIter];
			double error = pcl::pointToPlaneDistance(nextPoint, //planeParameters);
								 planeCoeffs->values[0],
								 planeCoeffs->values[1],
								 planeCoeffs->values[2],
								 planeCoeffs->values[3]);
			sumSquaredErrors += error*error;
			sumAbsErrors += error;
			double distanceFromCentroid = euclideanDistance(centroid, nextPoint);
			ssTot += distanceFromCentroid*distanceFromCentroid;

			maxError = max(error, maxError);
			minError = min(error, minError);
			maxDist = max(distanceFromCentroid, maxDist);
			minDist = min(distanceFromCentroid, minDist);
		}


		//------compute R^2
		double meanError = sumAbsErrors/planeIndices->indices.size();
		cout << endl << " | sum^2 errors = " << sumSquaredErrors
			 << " | mean error = " << meanError
			 << " | max error = " << maxError << " | minError = " << minError << endl
			 << " | maxDistFromCentroid = " << maxDist << " | minDistFromCentroid = " << minDist << endl;

		return 1 - (sumSquaredErrors/ssTot);
	}


  PointIndices::Ptr Segmentation::fitCylinderNew(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
					      boost::shared_ptr<set<int> >  subcloudIndices,
					      double &x, double &y, double &z,
					      double &roll, double &pitch, double &yaw, 
					      double &radius,
					      double &length)
  {
    PointCloud<PointXYZRGB>::Ptr subcloud = PclSurrogateUtils::extractIndexedPoints(subcloudIndices, cloud);

    //---normals
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
    NormalEstimation<PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (subcloud);
    ne.setKSearch (50);
    PointCloud<pcl::PointXYZRGBNormal>::Ptr subcloud_normals (new PointCloud<pcl::PointXYZRGBNormal>);
    ne.compute (*subcloud_normals);

    pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB,pcl::PointXYZRGBNormal>::Ptr model_cyl(new 
	     pcl::SampleConsensusModelCylinder<pcl::PointXYZRGB,pcl::PointXYZRGBNormal>(subcloud));
    model_cyl->setRadiusLimits(0.01,0.5);
    model_cyl->setInputNormals(subcloud_normals);
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_cyl);
    ransac.setDistanceThreshold(0.09);
    ransac.computeModel();
    PointIndices::Ptr cylIndices (new PointIndices); //TODO set
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    cout << "SampleConsensusModelCylinder: ";
    for(int i=0;i<coeff.size();i++) cout << coeff[i] << ", ";
    cout << endl;

    x=coeff[0];
    y=coeff[1];
    z=coeff[2];
    float dx=coeff[3];
    float dy=coeff[4];
    float dz=coeff[5];
    if(dz<0){
      dx=-dx;
      dy=-dy;
      dz=-dz;
    }
    roll = 0;
    pitch = acos(dz);
    yaw = atan2(dy, dx)-M_PI;
    radius = coeff[6];

    cout << length << endl;

    return cylIndices;


  }


  //==============cylinder
  /**fits a cylinder to subcloudIndices in cloud.  currently, we assume
     the cylinder is oriented on the z axis*/
  PointIndices::Ptr Segmentation::fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
					      boost::shared_ptr<set<int> >  subcloudIndices,
					      const FittingParams& fp,
					      double &x, double &y, double &z,
					      double &roll, double &pitch, double &yaw, 
					      double &radius,
					      double &length,  std::vector<double> & inliers_distances)
  {
    cout << "\n in fit cylinder.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;

    PointCloud<PointXYZRGB>::Ptr subcloud = PclSurrogateUtils::extractIndexedPoints(subcloudIndices, cloud);


    pcl::PCDWriter writer;

    //debugging
    Eigen::Vector4f centroid4f;
    compute3DCentroid(*subcloud, centroid4f);
    PointXYZ centroid(centroid4f[0], centroid4f[1], centroid4f[2]);
    
    cout << "\n subcloud centroid = " << centroid << endl;
    //end debugging

    //---normals
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
    NormalEstimation<PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (subcloud);
    ne.setKSearch (50);
    PointCloud<pcl::Normal>::Ptr subcloud_normals (new PointCloud<pcl::Normal>);
    ne.compute (*subcloud_normals);

    //create the segmentation object
    SACSegmentationFromNormals<PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(SACMODEL_CYLINDER); //pcl::SACMODEL_CYLINDER);
    seg.setMethodType(SAC_MLESAC); 
    seg.setDistanceThreshold(fp.distanceThreshold); //0.05);
    seg.setRadiusLimits(fp.minRadius, fp.maxRadius); //0.05, 0.2);

    //set input
    seg.setInputCloud(subcloud);
    seg.setInputNormals(subcloud_normals);

    // convert FittingParams YPR to XYZ vector
    Matrix3f Rx,Ry,Rz;
    Rx << 1,0,0, 0,cos(fp.roll),-sin(fp.roll), 0,sin(fp.roll),cos(fp.roll);
    Ry << cos(fp.pitch),0,sin(fp.pitch), 0,1,0, -sin(fp.pitch),0,cos(fp.pitch);
    Rz << cos(fp.yaw),-sin(fp.yaw),0, sin(fp.yaw),cos(fp.yaw),0, 0,0,1;
    Vector3f seedDirection = Rz*Ry*Rx*Vector3f(0,0,1);

    //segment
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr cylinderIndices (new PointIndices);
    seg.setAxis(seedDirection); 
    seg.setEpsAngle(fp.maxAngle); // seg faults if too small
    seg.segment(*cylinderIndices, *coefficients);

    cout << "Cylinder: ";
    for(int i=0;i<coefficients->values.size();i++){
      cout<<coefficients->values[i]<<", ";
    }
    cout<<endl;
    // TODO can coeff be empty?

    // calculate roll, pitch, and yaw
    Vector3f base(&coefficients->values[0]);
    Vector3f direction(&coefficients->values[3]);
    // flip if z is negative to simplify math
    if(direction.z()<0) direction = -direction;
    roll = 0;
    pitch = acos(direction.z());
    yaw = atan2(direction.y(), direction.x());
    radius = coefficients->values[6];
    
    writer.write ("table_objects.pcd", *subcloud, false);

    cout << "\n segmentation coefficients:\n" << *coefficients << endl;
		
    // find direction's largest component x, y, or z
    int maxIndex = 0;
    for(int i=1;i<3;i++) if(fabs(direction[i]) > fabs(direction[maxIndex])) maxIndex=i;

    // project points on to line and find endpoints
    Vector3f pMin, pMax;    
    for(int i=0; i<cylinderIndices->indices.size();i++){ // for each inlier
      // extract point
      int index = cylinderIndices->indices[i];
      PointXYZRGB& pt = subcloud->at(index);
      Vector3f q(pt.x,pt.y,pt.z);
      
      // project on to line
      Vector3f pq = q - base;
      Vector3f w2 = pq - direction * pq.dot(direction);
      Vector3f p = q - w2;

      // find end points
      if(i==0){
				pMin = pMax = p;
      }else{ 
				if(p[maxIndex]<pMin[maxIndex]) pMin = p;
				if(p[maxIndex]>pMax[maxIndex]) pMax = p;
      }
    }

    // copy results to output
    Vector3f center = (pMax+pMin)/2;
    x = center.x();
    y = center.y();
    z = center.z();
    length = (pMax-pMin).norm();

    // residuals 
    inliers_distances.clear ();
    inliers_distances.resize (cylinderIndices->indices.size ());
    Eigen::Vector4f line_pt  (coefficients->values[0], coefficients->values[1], coefficients->values[2], 0);
    Eigen::Vector4f line_dir (coefficients->values[3], coefficients->values[4], coefficients->values[5], 0);

    for (size_t i = 0; i < cylinderIndices->indices.size (); ++i)
      {
				Eigen::Vector4f pt (subcloud->points[cylinderIndices->indices[i]].x, subcloud->points[cylinderIndices->indices[i]].y, subcloud->points[cylinderIndices->indices[i]].z, 0);
        double d_euclid = fabs (sqrt(pcl::sqrPointToLineDistance (pt, line_pt, line_dir)) - coefficients->values[6]);
				inliers_distances[i] = d_euclid; 
      }
		
    return cylinderIndices;
  }

  //==============sphere
  PointIndices::Ptr Segmentation::fitSphere(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
					    boost::shared_ptr<set<int> >  subcloudIndices,
					    const FittingParams& fp,
					    double &x, double &y, double &z,
					    double &radius)
  {
    cout << "\n in fit sphere.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;

    PointCloud<PointXYZRGB>::Ptr subcloud = PclSurrogateUtils::extractIndexedPoints(subcloudIndices, cloud);

    //debugging
    Eigen::Vector4f centroid4f;
    compute3DCentroid(*subcloud, centroid4f);
    PointXYZ centroid(centroid4f[0], centroid4f[1], centroid4f[2]);
    
    cout << "\n subcloud centroid = " << centroid << endl;
    //end debugging

    //---normals
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
    NormalEstimation<PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (subcloud);
    ne.setKSearch (50);
    PointCloud<pcl::Normal>::Ptr subcloud_normals (new PointCloud<pcl::Normal>);
    ne.compute (*subcloud_normals);


#if 1
    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr model_sphere(new 
									pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>(subcloud));
    model_sphere->setRadiusLimits(fp.minRadius, fp.maxRadius);
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_sphere);
    ransac.setDistanceThreshold(fp.distanceThreshold);
    //ransac.setSampleConsensusModel(pcl::SAC_MLESAC);
    ransac.computeModel();
    PointIndices::Ptr sphereIndices (new PointIndices); //TODO set
    /*vector<int> inliers;
      ransac.getInliers(inliers);
      PointCloud<PointXYZRGB>::Ptr outputcloud
      pcl::copyPointCloud<pcl::pointXYZRGB>(*subcloud,inliers,*outputcloud);*/
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    cout << "SampleConsensusModelSphere: ";
    for(int i=0;i<coeff.size();i++) cout << coeff[i] << ", ";
    cout << endl;

    if(coeff.size()==4){
      x = coeff[0];
      y = coeff[1];
      z = coeff[2];
      radius = coeff[3];
    }else{
      //TODO can this happen?
    }
    return sphereIndices;

#else
    //create the segmentation object
    SACSegmentationFromNormals<PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(SACMODEL_SPHERE); //pcl::SACMODEL_CYLINDER);
    seg.setMethodType(SAC_MLESAC); 
    seg.setDistanceThreshold(0.09); //0.05);
    seg.setRadiusLimits(0.01, 0.25); //0.05, 0.2);
       
    //set input
    seg.setInputCloud(subcloud);
    seg.setInputNormals(subcloud_normals);

    //segment
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr sphereIndices (new PointIndices);
    seg.segment(*sphereIndices, *coefficients);
    
    //todo: xyz / rpyaw  / radius, length
    //model_coefficientsthe coefficients of the sphere (center, sphere_radius_R)
    x = coefficients->values[0];
    y = coefficients->values[1];
    z = coefficients->values[2];    
    radius = coefficients->values[6];
    
    cout << "\n segmentation coefficients:\n" << *coefficients << endl;

    //hack: using centroid.z since the coeffients.z could be anywhere on the axis (assuming cylinder
    //is oriented vertically)
    z = centroid.z;

    return sphereIndices;
#endif
  }


  PointIndices::Ptr Segmentation::fitCircle3d(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
					      boost::shared_ptr<set<int> >  subcloudIndices,
					      const FittingParams& fp,
					      double &x, double &y, double &z,
					      double &roll, double &pitch, double &yaw, 
					      double &radius,
					      std::vector<double> & inliers_distances)
  {
    cout << "\n in fit cylinder.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;
		
    PointCloud<PointXYZRGB>::Ptr subcloud = PclSurrogateUtils::extractIndexedPoints(subcloudIndices, cloud);

    SampleConsensusModelCircle3D<PointXYZRGB>::Ptr model (new SampleConsensusModelCircle3D<PointXYZRGB> (subcloud));
		
    // Create the RANSAC object
    RandomSampleConsensus<PointXYZRGB> sac(model, 0.01);
    // Algorithm tests
    bool result = sac.computeModel ();

    cout << "convergence: " << result << endl;

    std::vector<int> sample;
    sac.getModel (sample);
		
    std::vector<int> inliers;
    sac.getInliers (inliers);
		
    Eigen::VectorXf coeff;
    sac.getModelCoefficients (coeff);

    x = coeff[0];
    y = coeff[1];
    z = coeff[2];
    radius = coeff[3];

    // convert direction to ypr
    Vector3f base(&coeff[0]);
    Vector3f direction(&coeff[4]);  
    // flip if z is negative to simplify math
    if(direction.z()<0) direction = -direction;
    roll = 0;
    pitch = acos(direction.z());
    yaw = atan2(direction.y(), direction.x());

    //TODO
    // inliers
    // fitting params
    // inlier distance
    // set inliers
    PointIndices::Ptr circle3dIndices (new PointIndices);

    return circle3dIndices;


  }

  //==============plane
  /**fits a plane to subcloudIndices in cloud.  currently, we assume
     the plane is oriented on the z axis*/
  PointIndices::Ptr Segmentation::fitPlane(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
					      boost::shared_ptr<set<int> >  subcloudIndices,
								const FittingParams& fp,
					      double &x, double &y, double &z,
					      double &roll, double &pitch, double &yaw, 
					      double &width,
					      double &length,  std::vector<double> & inliers_distances)
  {
    cout << "\n in fit plane.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;


    PointCloud<PointXYZRGB>::Ptr subcloud = PclSurrogateUtils::extractIndexedPoints(subcloudIndices, cloud);

    //debugging
    //Eigen::Vector4f centroid4f;
    //compute3DCentroid(*subcloud, centroid4f);
    //PointXYZ centroid(centroid4f[0], centroid4f[1], centroid4f[2]);
    
    //cout << "\n subcloud centroid = " << centroid << endl;
    //end debugging

		PointIndices::Ptr subcloudIndicesCopy = PclSurrogateUtils::toPclIndices(subcloudIndices); //PclSurrogateUtils::copyIndices(subcloudIndices);

    //---normals
    pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB> ());
    NormalEstimation<PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (subcloud);
    ne.setKSearch (50);
    PointCloud<pcl::Normal>::Ptr subcloud_normals (new PointCloud<pcl::Normal>);
    ne.compute (*subcloud_normals);

    //create the segmentation object
    SACSegmentationFromNormals<PointXYZRGB, pcl::Normal> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); 
		seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (fp.distanceThreshold);

    //set input
    seg.setInputCloud(subcloud);
    seg.setInputNormals(subcloud_normals);

		// convert FittingParams YPR to XYZ vector
		Matrix3f Rx,Ry,Rz;
		Rx << 1,0,0, 0,cos(fp.roll),-sin(fp.roll), 0,sin(fp.roll),cos(fp.roll);
		Ry << cos(fp.pitch),0,sin(fp.pitch), 0,1,0, -sin(fp.pitch),0,cos(fp.pitch);
		Rz << cos(fp.yaw),-sin(fp.yaw),0, sin(fp.yaw),cos(fp.yaw),0, 0,0,1;
		Vector3f seedDirection = Rz*Ry*Rx*Vector3f(0,0,1);
		Vector3f thetaSeed =  Rz*Ry*Rx*Vector3f(1,0,0);  //TODO use this later

    //segment
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr planeIndices (new PointIndices);
		seg.setAxis(seedDirection); 
		seg.setEpsAngle(fp.maxAngle); // seg faults if too small
    seg.segment(*planeIndices, *coefficients);

    cout << "Plane: ";
    for(int i=0;i<coefficients->values.size();i++){
      cout<<coefficients->values[i]<<", ";
    }
    cout<<endl;
    // TODO can coeff be empty?

    // calculate normal 
    // plane: ax+by+cz+d=0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];
    Vector3f normal(a,b,c);
    normal.normalize();

    //writer.write ("table_objects.pcd", *subcloud, false);

    cout << "\n segmentation coefficients:\n" << *coefficients << endl;
		
    // iterate through possible rotations around normal to find best fit
    Vector3f center,ypr;
    Vector2f lengthWidth;
    float minArea=numeric_limits<float>::max();
    float bestTheta;
    // TODO 3 passes with variable resolution
    for(float theta=-180;theta<180;theta++){
      lengthWidth = getLengthWidth(subcloud, planeIndices, coefficients->values.data(), normal, 
				   theta*M_PI/180.0f, center,ypr);
      float area = lengthWidth[0]*lengthWidth[1];
      if(area<minArea){
	minArea = area;
	bestTheta=theta;
      }
    }
    lengthWidth = getLengthWidth(subcloud, planeIndices, coefficients->values.data(), normal, 
				 bestTheta*M_PI/180.0f, center,ypr);
    
    
    // copy to output
    x = center.x();
    y = center.y();
    z = center.z();
    yaw = ypr[0];
    pitch = ypr[1];
    roll = ypr[2];
    length = lengthWidth[0];
    width = lengthWidth[1];
    
    cout << "xyz_rpy_w_l:" << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << width << " " << length << endl;
    
    
    
    
    // residuals 
    /* TODO
			inliers_distances.clear ();
    inliers_distances.resize (planeIndices->indices.size ());
    Eigen::Vector4f line_pt  (coefficients->values[0], coefficients->values[1], coefficients->values[2], 0);
    Eigen::Vector4f line_dir (coefficients->values[3], coefficients->values[4], coefficients->values[5], 0);

    for (size_t i = 0; i < planeIndices->indices.size (); ++i)
      {
	Eigen::Vector4f pt (subcloud->points[planeIndices->indices[i]].x, subcloud->points[planeIndices->indices[i]].y, subcloud->points[planeIndices->indices[i]].z, 0);
        double d_euclid = fabs (sqrt(pcl::sqrPointToLineDistance (pt, line_pt, line_dir)) - coefficients->values[6]);
	inliers_distances[i] = d_euclid; 
	}*/

		/* TODO
			 - compute orientation
			   - create function [width,length] = computeWidthLength(cloud, yaw)
				 - compute orientation in 3 passes: -180:10:180, -10:1:10, -1:0.1:1
			 - support multiple planes
			 - compute residuals
			 - final residuals is min of residuals for each plane
			 - create new plane struct
			 - return vector of planes
			 - add number of instances to gui
		*/

    return planeIndices;
  }






#if 0
    uint min_plane_size = (int) (0.2 * subcloud->points.size());
    if (min_plane_size > 500 || min_plane_size < 300)
      min_plane_size = 500;

    int maxNumSegments = 10; 

    for (uint i = 0; i < maxNumSegments && subcloud->points.size() > min_plane_size; i++){

      seg.setInputCloud(subcloud);
      seg.setIndices(subcloudIndicesCopy);

      ModelCoefficients::Ptr coefficients(new ModelCoefficients);
      PointIndices::Ptr nextSegmentIndices (new PointIndices);
      seg.segment(*nextSegmentIndices, *coefficients);

      /*if (nextSegmentIndices->indices.size() > min_plane_size)
	segmentsFound.push_back(nextSegmentIndices); 
      else
	break;
      */

      boost::shared_ptr<set<int> > remainingIndices (new set<int>(subcloudIndicesCopy->indices.begin(), subcloudIndicesCopy->indices.end()));

      for(uint i = 0; i < nextSegmentIndices->indices.size(); i++)
	remainingIndices->erase(nextSegmentIndices->indices[i]);

      subcloudIndicesCopy = PclSurrogateUtils::toPclIndices(remainingIndices);
    }


    ///////////////////// FIXME //////////////////////////////////// 
    PointIndices::Ptr planeIndices (new PointIndices);
    return planeIndices;

}


#endif  

/* calculates bounding box, center, and orientation of plane given theta
 * Rotates points to XY plane finds bounds and center, rotates center back to original rotation frame
 */
Vector2f Segmentation::getLengthWidth(PointCloud<PointXYZRGB>::Ptr subcloud, PointIndices::Ptr planeIndices, 
				      float plane[4],Vector3f normal,float theta,
				      Vector3f& center, Vector3f& ypr){
  
  float a = plane[0];
  float b = plane[1];
  float c = plane[2];
  float d = plane[3];
  
  // compute rotation matrix to horizontal
  Vector3f axis = normal.cross(Vector3f(0,0,1)).normalized();
  float angle = acos(normal.dot(Vector3f(0,0,1)));
  Matrix3f rot;
  rot = AngleAxisf(theta,Vector3f(0,0,1))*AngleAxisf(angle,axis);
  
  float minX,maxX,minY,maxY;
  float zh;
  
  // compute length line
  for(int i=0; i<planeIndices->indices.size();i++){ // for each inlier
    
    // extract point and project to plane
    int index = planeIndices->indices[i];
    PointXYZRGB& pt = subcloud->at(index);
    float correctedZ = -(a*pt.x + b*pt.y + d)/c; // move z to plane
    Vector3f p(pt.x,pt.y,correctedZ);
    
    // rotate to horizontal plane
    Vector3f ph = rot*p;
    
    // find extremes
    if(i==0){
      minX=maxX=ph[0];
      minY=maxY=ph[1];
      zh = ph[2];
    }else{
      minX=min(ph[0],minX);
      maxX=max(ph[0],maxX);
      minY=min(ph[1],minY);
      maxY=max(ph[1],maxY);
    }
  }
  
  Vector2f lengthWidth(maxY-minY,maxX-minX);
  
  center = Vector3f( (minX+maxX)/2, (maxY+minY)/2, zh );
  center = rot.inverse()*center;
  ypr = rot2ypr(rot);
  
  return lengthWidth;     
  
}


	//==================
	//---------non-instantiable
	Segmentation::Segmentation()
	{
		throw exception();
	}

	Segmentation::~Segmentation()
	{
		throw exception();
	}


} //namespace surrogate_gui
