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
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pointcloud_tools/filter_planes.hpp>

#include <bot_core/rotations.h>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace drc::PointConvert;

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
    bot_matrix_to_quat(mat3,q);

    // in some cases, q is NaN.  Try to fix it. HACK //TODO do better fix
    // ususally caused by numbers near 0 and 1
    if(std::isnan(q[0])){
      cout << "***** Warning quat is NaN, trying to fix.  Matrix:\n"<< mat << endl;
      for(int i=0;i<9;i++) mat3[i] = round(mat3[i]);
      bot_matrix_to_quat(mat3,q);
      if(std::isnan(q[0])) cout << "Couldn't fix\n";
      else            cout << "Fix seemed to work.\n";
    }

    bot_quat_to_roll_pitch_yaw(q,rpy);
    return Vector3f(rpy[2],rpy[1],rpy[0]);
  }

  void remove_xyzypr(const PointCloud<PointXYZRGB>::ConstPtr cloud,
                     PointIndices::Ptr subcloudIndices,
                     Vector3f xyz, Vector3f ypr, 
                     vector<vector<float> >& points)
  {
    //point cloud indices
    Matrix3f rot = ypr2rot(ypr).inverse(); 
    points.resize(subcloudIndices->indices.size());
    for(int i=0;i<points.size();i++){
      int index = subcloudIndices->indices[i];
      const PointXYZRGB& ptc = cloud->at(index);
      Vector3f pt(ptc.x,ptc.y,ptc.z);
      
      // remove objects xyzypr from each inlier point and add to affordanceMsg
      pt -= xyz;
      pt = rot*pt;
      points[i].resize(3);
      points[i][0] = pt[0];
      points[i][1] = pt[1];
      points[i][2] = pt[2];
    }
  }
               
  void remove_xyzypr(vector<Vector3f>& points, Vector3f xyz, Vector3f ypr)
  {
    //point cloud indices
    // remove objects xyzypr from each inlier point and add to affordanceMsg
    Matrix3f rot = ypr2rot(ypr).inverse(); 
    for(int i=0;i<points.size();i++){
      points[i] -= xyz;
      points[i] = rot*points[i];
    }
  }

  PointCloud<PointXYZRGB>::Ptr extractAndSmooth(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, 
                                                boost::shared_ptr<set<int> >  subcloudIndices 
                                                   = boost::shared_ptr<set<int> >())
  {
    PointCloud<PointXYZRGB>::Ptr subcloud;
    if(!subcloudIndices || subcloudIndices->size()==0) subcloud.reset(new PointCloud<PointXYZRGB>(*cloud));  
    else subcloud = PclSurrogateUtils::extractIndexedPoints(subcloudIndices, cloud);

    return subcloud;

    /*pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    
    mls.setComputeNormals (true);
    
    // Set parameters
    mls.setInputCloud (subcloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.15);
    
    // Reconstruct
    mls.process (*mls_points);

    return mls_points;*/
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
/*    cout << "\n PCL 1.5.1 throws seg fault when trying to euclidean cluster.  skipping euclidean clusterin"
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


  //==============cylinder
  /**fits a cylinder to subcloudIndices in cloud.  currently, we assume
     the cylinder is oriented on the z axis*/
  PointIndices::Ptr Segmentation::fitCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                boost::shared_ptr<set<int> >  subcloudIndices,
                const FittingParams& fp,
                double &x, double &y, double &z,
                double &roll, double &pitch, double &yaw, 
                double &radius,
                double &length,  
                std::vector<Eigen::Vector3f>& points,
                std::vector<Eigen::Vector3i>& triangles,
                std::vector<double> & inliers_distances)
  {
    cout << "\n in fit cylinder.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;

    PointCloud<PointXYZRGB>::Ptr subcloud = extractAndSmooth(cloud, subcloudIndices);

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

    // remove xyzypr from inliers and copy to output
    if(true){
      vector<vector<float> > pointsV;
      remove_xyzypr(subcloud, cylinderIndices, Vector3f(x,y,z), 
                                         Vector3f(yaw,pitch,roll), pointsV);
      points.resize(pointsV.size());
      for(int i=0;i<pointsV.size();i++){
        points[i][0] = pointsV[i][0];
        points[i][1] = pointsV[i][1];
        points[i][2] = pointsV[i][2];
      }
    } else {
      // compute triangles
      cloudToTriangles(subcloud, cylinderIndices, points, triangles);
      remove_xyzypr(points, Vector3f(x,y,z), Vector3f(yaw,pitch,roll));
    }

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
                                            double &radius,
                                            std::vector< std::vector<float> > &inliers)
  {
    cout << "\n in fit sphere.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;

    PointCloud<PointXYZRGB>::Ptr subcloud = extractAndSmooth(cloud, subcloudIndices);

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

    pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr model_sphere(new 
                         pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>(subcloud));
    model_sphere->setRadiusLimits(fp.minRadius, fp.maxRadius);
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_sphere);
    ransac.setDistanceThreshold(fp.distanceThreshold);
    //ransac.setSampleConsensusModel(pcl::SAC_MLESAC);
    ransac.computeModel();
    PointIndices::Ptr sphereIndices (new PointIndices); 
    ransac.getInliers(sphereIndices->indices);
    /*PointCloud<PointXYZRGB>::Ptr outputcloud
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
    // remove xyzypr from inliers and copy to output
    remove_xyzypr(subcloud, sphereIndices, Vector3f(x,y,z), Vector3f(0,0,0), inliers);

    return sphereIndices;

  }


  PointIndices::Ptr Segmentation::fitCircle3d(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                              boost::shared_ptr<set<int> >  subcloudIndices,
                                              const FittingParams& fp,
                                              double &x, double &y, double &z,
                                              double &roll, double &pitch, double &yaw, 
                                              double &radius,
                                              std::vector< std::vector<float> > &inliers,
                                              std::vector<double> & inliers_distances)
  {
    cout << "\n in fit cylinder.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;
                
    PointCloud<PointXYZRGB>::Ptr subcloud = extractAndSmooth(cloud, subcloudIndices);

    SampleConsensusModelCircle3D<PointXYZRGB>::Ptr model (new SampleConsensusModelCircle3D<PointXYZRGB> (subcloud));
                
    // Create the RANSAC object
    RandomSampleConsensus<PointXYZRGB> sac(model, 0.01);
    // Algorithm tests
    bool result = sac.computeModel ();

    cout << "convergence: " << result << endl;

    std::vector<int> sample;
    sac.getModel (sample);
                
    PointIndices::Ptr circle3dIndices (new PointIndices);
    sac.getInliers (circle3dIndices->indices);
                
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
    // fitting params
    // inlier distance

    // remove xyzypr from inliers and copy to output
    remove_xyzypr(subcloud, circle3dIndices, Vector3f(x,y,z), Vector3f(yaw,pitch,roll), inliers);

    return circle3dIndices;


  }

  //==============plane
  /**fits a plane to subcloudIndices in cloud.  currently, we assume
     the plane is oriented on the z axis*/
  void Segmentation::fitPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                               boost::shared_ptr<set<int> >  subcloudIndices,
                               const FittingParams& fp,
                               vector<Plane>& planeList)
  { 
    cout << "\n in fit plane.  num indices = " << subcloudIndices->size() << endl;
    cout << "\n cloud size = " << cloud->points.size() << endl;
    
    PointCloud<PointXYZRGB>::Ptr subcloud = extractAndSmooth(cloud, subcloudIndices);
    
    // 2. Extract the major planes and send them to lcm:
    int plane_fitter_id_ =1;
    int64_t current_utime = 0;
    FilterPlanes filtp;
    filtp.setInputCloud(subcloud);
    filtp.setPoseIDs(plane_fitter_id_,current_utime);
    //filtp.setLCM(lcm_->getUnderlyingLCM());
    filtp.setDistanceThreshold(0.02); // simulated lidar
    filtp.setStopProportion(0.020);  //was 0.1
    filtp.setStopCloudSize(50);
    vector<BasicPlane> plane_stack; 
    filtp.filterPlanes(plane_stack);
    std::cout << "[PLANE] number of planes extracted: " << plane_stack.size() << "\n";
    
    planeList.resize(plane_stack.size());
    for(int pi=0;pi<plane_stack.size();pi++){
      
      BasicPlane& bp = plane_stack[pi];  // input
      Plane& p = planeList[pi];          // output
      
      cout << "Plane: ";
      for(int i=0; i<bp.coeffs.values.size(); i++){  
        cout<<bp.coeffs.values[i]<<", ";
      }
      cout<<endl;
      // TODO can coeff be empty?
      
      // calculate normal 
      // plane: ax+by+cz+d=0
      float a = bp.coeffs.values[0];
      float b = bp.coeffs.values[1];
      float c = bp.coeffs.values[2];
      float d = bp.coeffs.values[3];
      Vector3f normal(a,b,c);
      normal.normalize();
      
      cout << "Convex hull has: " << bp.cloud.points.size () << " data points." << std::endl;
      p.convexHull.resize(bp.cloud.points.size());
      for(int i=0; i<p.convexHull.size(); i++){
        p.convexHull[i][0] = bp.cloud.points[i].x;
        p.convexHull[i][1] = bp.cloud.points[i].y;
        p.convexHull[i][2] = bp.cloud.points[i].z;
      }

    cout << "\n segmentation coefficients:\n" << bp.coeffs << endl;

    // copy to output
    Vector4f centroid;
    compute3DCentroid(bp.cloud, centroid);
    Vector3f center(centroid[0],centroid[1],centroid[2]);
    p.xyz = center;
    p.ypr = Vector3f( atan2(normal[1],normal[0]), acos(normal[2]/normal.norm()), 0 );
    p.length = 0;//lengthWidth[0];
    p.width = 0;//lengthWidth[1];

    // remove xyzypr from inliers and copy to output
    remove_xyzypr(p.convexHull, p.xyz, p.ypr);

    //TODO
    // compute inlier
    // compute inlier distances (or remove from struct)
    
    cout << "xyz_rpy_w_l:" << p.xyz << " " << p.ypr << p.width << " " << p.length << endl;
    
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
    }
    //return planeIndices;
  }

  //convenient typedefs
  typedef pcl::PointXYZRGB MyPointT;
  typedef pcl::PointCloud<MyPointT> MyPointCloud;
  typedef pcl::PointNormal MyPointNormalT;
  typedef pcl::PointCloud<MyPointNormalT> MyPointCloudWithNormals;
  
  void pairAlign (const MyPointCloud::Ptr cloud_src, const MyPointCloud::Ptr cloud_tgt, MyPointCloud::Ptr output, Eigen::Matrix4f &final_transform, int nbIteration, int ICPIter, bool downsample = false)
{
    MyPointCloud::Ptr src (new MyPointCloud);
    MyPointCloud::Ptr tgt (new MyPointCloud);
    pcl::VoxelGrid<MyPointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);
 
        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }
 
    MyPointCloudWithNormals::Ptr points_with_normals_src (new MyPointCloudWithNormals);
    MyPointCloudWithNormals::Ptr points_with_normals_tgt (new MyPointCloudWithNormals);
 
    pcl::NormalEstimation<MyPointT, MyPointNormalT> norm_est;
    pcl::search::KdTree<MyPointT>::Ptr tree (new pcl::search::KdTree<MyPointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);
 
    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);
 
    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
 
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    MyPointCloudWithNormals::Ptr reg_result = points_with_normals_src;
 
    pcl::IterativeClosestPoint<MyPointNormalT, MyPointNormalT> reg;
    reg.setMaximumIterations (ICPIter);
    reg.setInputTarget (points_with_normals_tgt);
 
    for (int i = 0; i < nbIteration; ++i)
    {
        PCL_INFO ("Iteration Nr. %d.\n", i);
 
        // save cloud for visualization purpose
        points_with_normals_src = reg_result;
 
        // Estimate
        reg.setInputCloud (points_with_normals_src);
        reg.align (*reg_result);
        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;
        //check if threshold is reach, if so decrease the threshold
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ()) 
           reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
 
        prev = reg.getLastIncrementalTransformation ();
    }
 
    // Get the transformation from target to source
    targetToSource = Ti;
 
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);
    //add the source to the transformed target
    *output += *cloud_src;
 
    final_transform = targetToSource;
}


  void Segmentation::fitPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                   boost::shared_ptr<set<int> >  subcloudIndices,
                                   const FittingParams& fp,
                                   bool isInitialSet, Vector3f initialXYZ, 
                                   Vector3f initialYPR,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelcloud,
                                   Vector3f& xyz, Vector3f& ypr,
                                   vector<pcl::PointCloud<pcl::PointXYZRGB> >& clouds)
  {

    PointCloud<PointXYZRGB>::Ptr subcloud = extractAndSmooth(cloud, subcloudIndices);
    Vector4f centroid;
    compute3DCentroid(*cloud, centroid);

    // icp
#if 1
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations (20);
    //icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setTransformationEpsilon (0.003);

    //icp.setRANSACOutlierRejectionThreshold (1);
    //icp.setTransformationEpsilon (0.001);
    //icp.setMaximumIterations (10);

    icp.setInputCloud(modelcloud);
    icp.setInputTarget(subcloud);
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    Matrix4f guess = Matrix4f::Identity();
    cout << "Initial guess XYZYPR: " << initialXYZ.transpose() << " " 
         << initialYPR.transpose() << endl;
    if(isInitialSet){
      Matrix3f rot = ypr2rot(initialYPR);
      guess.block<3,3>(0,0) = rot;
      guess(0,3) = initialXYZ[0];
      guess(1,3) = initialXYZ[1];
      guess(2,3) = initialXYZ[2];
    }
    icp.align(Final, guess);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;
    Matrix4f transformation = icp.getFinalTransformation();
    //std::cout << transformation << std::endl;
    
#else

    // pair align
    PointCloud<PointXYZRGB>::Ptr output(new PointCloud<PointXYZRGB>);
    Matrix4f transformation = Matrix4f::Identity();
    pairAlign(modelcloud, subcloud, output, transformation, 10, 2, true);

#endif
    cout << "Max corrs: " << icp.getMaxCorrespondenceDistance () << endl;
    cout << "Max iter: " << icp.getMaximumIterations () << endl;
    cout << "Max trans eps: " << icp.getTransformationEpsilon () << endl;
    cout << "Max euclid eps: " << icp.getEuclideanFitnessEpsilon () << endl;

    clouds.push_back(*modelcloud);
    clouds.push_back(*subcloud);
    clouds.push_back(Final);

    // extract xyzrpy
    Matrix3f rot = transformation.block<3,3>(0,0);
    xyz = transformation.block<3,1>(0,3);
    ypr = rot2ypr(rot);
    
    cout << "Guess:\n" << guess << endl;
    cout << "Matrix:\n" << transformation << endl;
    cout << "xyz_ypr: " << xyz.transpose() << " " << ypr.transpose() << endl;

  }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segmentation::subSampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, float voxel_grid_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>); 
  vox_grid.filter (*tempCloud);
  return tempCloud; 
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Segmentation::computeFpfh(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, float normalRadius, float fpfhRadius)
{
  ////////////////////////////////////////
  // compute fpfh TODO: only compute once


  // compute normals
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (normalRadius);
  ne.compute (*normals);

  // Create the FPFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud (cloud);
  fpfh.setInputNormals (normals);

  /*{
    pcl::visualization::PCLVisualizer pclvis;
    pclvis.addPointCloud<pcl::PointXYZRGB>(cloud,"cloud");
    pclvis.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,1,0.02,"normal");
    pclvis.setBackgroundColor (0, 0, 0);  
    pclvis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    pclvis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normal");
    pclvis.initCameraParameters ();
    
    while (!pclvis.wasStopped ()) pclvis.spinOnce (100);

  }*/

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<PointXYZRGB>);

  fpfh.setSearchMethod (tree2);

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  fpfh.setRadiusSearch (fpfhRadius);

  // Compute the features
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>);
  fpfh.compute (*fpfhs);
  return fpfhs;
}

void Segmentation::fitPointCloudFpfh(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                 boost::shared_ptr<set<int> >  subcloudIndices,
                                 const FittingParams& fp,
                                 bool isInitialSet, Vector3f initialXYZ, 
                                 Vector3f initialYPR,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelcloud,
                                 Vector3f& xyz, Vector3f& ypr,
                                 vector<pcl::PointCloud<pcl::PointXYZRGB> >& clouds,
                                 FpfhParams params)
{

  // subsample
  PointCloud<PointXYZRGB>::Ptr subcloud = subSampleCloud(extractAndSmooth(cloud, subcloudIndices), params.gridSize);
  PointCloud<PointXYZRGB>::Ptr submodelcloud = subSampleCloud(modelcloud, params.gridSize);
  //PointCloud<PointXYZRGB>::Ptr subcloud = extractAndSmooth(cloud, subcloudIndices);
  //PointCloud<PointXYZRGB>::Ptr submodelcloud = modelcloud;

  // create guess matrix
  Matrix4f guess = Matrix4f::Identity();
  cout << "Initial guess XYZYPR: " << initialXYZ.transpose() << " " 
       << initialYPR.transpose() << endl;
  if(isInitialSet){
    Matrix3f rot = ypr2rot(initialYPR);
    guess.block<3,3>(0,0) = rot;
    guess(0,3) = initialXYZ[0];
    guess(1,3) = initialXYZ[1];
    guess(2,3) = initialXYZ[2];
  }

  // compute fpfh
  cout << "computing cloud fpfh\n";
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudFpfh = computeFpfh(subcloud, params.normalRadius, params.fpfhRadius);
  cout << "computing model fpfh\n";
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr modelFpfh = computeFpfh(submodelcloud, params.normalRadius,
                                                                   params.fpfhRadius);  //TODO only once
  cout << "Done\n";

  SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> scia;
  scia.setInputCloud(submodelcloud);
  scia.setSourceFeatures(modelFpfh);
  scia.setInputTarget(subcloud);
  scia.setTargetFeatures(cloudFpfh);
  scia.setMinSampleDistance (params.minSampleDistance);
  scia.setMaxCorrespondenceDistance (params.maxCorrespondenceDistance);
  scia.setMaximumIterations (params.maxIterations);
  scia.setNumberOfSamples(params.numberOfSamples);

  pcl::PointCloud<pcl::PointXYZRGB> registration_output;
  cout << "Aligning...\n";
  scia.align (registration_output);
  cout << "Done.\n";
  Matrix4f transformation = scia.getFinalTransformation();
 
  Matrix3f rot = transformation.block<3,3>(0,0);
  xyz = transformation.block<3,1>(0,3);
  ypr = rot2ypr(rot);
  
  cout << "Matrix:\n" << transformation << endl;
  cout << "xyz_ypr: " << xyz.transpose() << " " << ypr.transpose() << endl;
      
}

/* calculates bounding box, center, and orientation of plane given theta
 * Rotates points to XY plane finds bounds and center, rotates center back to original rotation frame
 */
Vector2f Segmentation::getLengthWidth(PointCloud<PointXYZRGB>& subcloud,  
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
  for(int i=0; i<subcloud.size(); i++){ // for each inlier
    
    // extract point and project to plane
    int index = i;
    PointXYZRGB& pt = subcloud.at(index);
    Vector3f p(pt.x,pt.y,pt.z);
    
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

  Matrix3f rotI = rot.inverse();
  center = Vector3f( (minX+maxX)/2, (maxY+minY)/2, zh );
  center = rotI*center;
  ypr = rot2ypr(rotI);
  
  return lengthWidth;     
  
}

  Vector2f Segmentation::findOptimalRectangle(PointCloud<PointXYZRGB>& cloud,  
                                          float plane[4], Vector3f& center, Vector3f& ypr){
    
    
    // iterate through possible rotations around normal to find best fit
    //Vector3f center,ypr;
    Vector2f lengthWidth;
    float minArea=numeric_limits<float>::max();
    float bestTheta;
    Vector3f normal(plane);

    // first pass: 10 degree increments
    for(float theta=-180;theta<180;theta+=10){
      lengthWidth = getLengthWidth(cloud, plane, normal, 
                                   theta*M_PI/180.0f, center,ypr);
      float area = lengthWidth[0]*lengthWidth[1];
      if(area<minArea){
        minArea = area;
        bestTheta=theta;
      }
    }

    // second pass: 1 degree increments
    for(float theta=bestTheta-10;theta<bestTheta+10;theta+=1){
      lengthWidth = getLengthWidth(cloud, plane, normal, 
                                   theta*M_PI/180.0f, center,ypr);
      float area = lengthWidth[0]*lengthWidth[1];
      if(area<minArea){
        minArea = area;
        bestTheta=theta;
      }
    }

    // third pass: 0.1 degree increments
    for(float theta=bestTheta-1;theta<bestTheta+1;theta+=0.1){
      lengthWidth = getLengthWidth(cloud, plane, normal, 
                                   theta*M_PI/180.0f, center,ypr);
      float area = lengthWidth[0]*lengthWidth[1];
      if(area<minArea){
        minArea = area;
        bestTheta=theta;
      }
    }

    lengthWidth = getLengthWidth(cloud, plane, normal, 
                                 bestTheta*M_PI/180.0f, center,ypr);
    
    return lengthWidth;
  }


  void Segmentation::cloudToTriangles(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                                      PointIndices::Ptr indices,
                                      std::vector<Eigen::Vector3f>& points,
                                      std::vector<Eigen::Vector3i>& triangles){

    // apply grid filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter_;
    voxel_grid_filter_.setLeafSize (0.0075, 0.0075, 0.0075);
    voxel_grid_filter_.setInputCloud (cloud_in);
    if(indices && indices->indices.size()!=0) voxel_grid_filter_.setIndices(indices);
    voxel_grid_filter_.filter (*cloud);  

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh polymesh;

    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.055);
    // Set typical values for the parameters
    gp3.setMaximumNearestNeighbors (100); // higher seems to make smaller triangles
    gp3.setMu (2.5);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (polymesh);

    // get points
    points.resize(cloud->size());
    for(int i=0;i<cloud->size();i++){
      points[i][0] = cloud->at(i).x;
      points[i][1] = cloud->at(i).y;
      points[i][2] = cloud->at(i).z;
    }

    // get triangles
    triangles.clear();
    for(int i=0;i<polymesh.polygons.size();i++){
      Vertices& v = polymesh.polygons[i];
      for(int j=0;j<v.vertices.size()-2;j++){
        Vector3i triangle(v.vertices[0],v.vertices[j+1],v.vertices[j+2]);
        triangles.push_back(triangle);
      }
    }
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
