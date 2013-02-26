#include "RendererBase.hpp"

#include <iostream>
#include <unordered_map>

#include <boost/circular_buffer.hpp>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <gtkmm.h>
#include <GL/gl.h>

#include <lcm/lcm-cpp.hpp>
#include <laser_utils/laser_util.h>
#include <bot_param/param_util.h>

#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core_planar_lidar_t.h>

#include "MeshRenderer.hpp"

using namespace maps;

struct LidarData {
  std::string mName;
  std::string mChannel;
  bool mActive;
  Eigen::Vector3f mColor;
  boost::shared_ptr<Laser_projector> mProjector;
  typedef boost::shared_ptr<laser_projected_scan> ScanPtr;
  typedef boost::circular_buffer<ScanPtr> ScanBuffer;
  ScanBuffer mScans;

  typedef boost::shared_ptr<LidarData> Ptr;

  LidarData() : mActive(false), mColor(0,0,0) {}

  ~LidarData() {
  }
};

class LidarTestRenderer : public RendererBase {
protected:
  enum DrawMode {
    DrawModePoints,
    DrawModeSurfelsWire,
    DrawModeSurfelsFilled
  };
  enum SurfelMode {
    SurfelModeFixed,
    SurfelModeDistance,
    SurfelModeAngle,
    SurfelModeNeighbors,
    SurfelModeHallucinate,
  };

  typedef std::unordered_map<std::string, LidarData::Ptr> LidarDataMap;
  LidarDataMap mDataMap;
  int mColorMode;
  int mDrawMode;
  int mSurfelMode;
  double mPointSize;
  double mSurfelScale;
  double mMinZ;
  double mMaxZ;
  int mScanHistory;
  boost::shared_ptr<maps::MeshRenderer> mMeshRenderer;
  std::vector<Eigen::Vector3f> mPointBuffer;
  
public:

  LidarTestRenderer(const std::string& iName,
                    BotViewer* iViewer, const int iPriority,
                    const lcm_t* iLcm,
                    const BotParam* iParam, const BotFrames* iFrames)
    : RendererBase(iName, iViewer, iPriority, iLcm, iParam, iFrames) {

    // set up ui widgets
    {
      std::vector<int> ids = {
        maps::MeshRenderer::ColorModeFlat,
        maps::MeshRenderer::ColorModeHeight,
        maps::MeshRenderer::ColorModeRange,
        maps::MeshRenderer::ColorModeCamera };
      std::vector<std::string> labels = { "Flat", "Height", "Range", "Camera"};
      mColorMode = maps::MeshRenderer::ColorModeHeight;
      addCombo("Color Mode", mColorMode, labels, ids);
    }
    {
      std::vector<int> ids = { DrawModePoints, DrawModeSurfelsWire,
                               DrawModeSurfelsFilled };
      std::vector<std::string> labels = { "Points", "Surfels Wire",
                                          "Surfels Filled"};
      mDrawMode = DrawModePoints;
      addCombo("Draw Mode", mDrawMode, labels, ids);
    }
    {
      std::vector<int> ids = { SurfelModeFixed, SurfelModeDistance,
                               SurfelModeAngle, SurfelModeNeighbors,
                               SurfelModeHallucinate };
      std::vector<std::string> labels = {"Fixed", "Distance", "Angle",
                                         "Neighbors", "Hallucinate" };
      mSurfelMode = SurfelModeFixed;
      addCombo("Surfel Size", mSurfelMode, labels, ids);
    }
    mPointSize = 3;      addSlider("Point Size", mPointSize, 0.1, 10, 0.1);
    mSurfelScale = 1;    addSlider("Surfel Scale", mSurfelScale, 0.1, 2.0, 0.1);
    mMinZ = 0;           addSlider("Z Scale Min", mMinZ, -1, 2, 0.01);
    mMaxZ = 1;           addSlider("Z Scale Max", mMaxZ, -1, 2, 0.01);
    mScanHistory = 1000; addSpin("Scan History", mScanHistory, 0, 1000, 1);

    // set up internal variables
    mMeshRenderer.reset(new MeshRenderer());
    mMeshRenderer->setLcm(getLcm());
    mMeshRenderer->setBotParam(getBotParam());
    mMeshRenderer->setCameraChannel("CAMERALEFT");

    // subscribe to lcm channels and set up data structures
    BotParam* botParam = getBotParam();
    char** lidarNames = bot_param_get_all_planar_lidar_names(botParam);
    if (lidarNames != NULL) {
      for (int i = 0; lidarNames[i] != NULL; ++i) {
        char prefix[1024];
        bot_param_get_planar_lidar_prefix(botParam, lidarNames[i], prefix,
                                          sizeof(prefix));
        std::string key = std::string(prefix) + ".lcm_channel";
        char* str;
        bot_param_get_str(botParam, key.c_str(), &str);
        std::string channel(str);
        free(str);
        add(getLcm()->subscribe(channel, &LidarTestRenderer::onLidar, this));

        key = std::string(prefix) + ".viewer_color";
        double color[3];
        bot_param_get_double_array(botParam, key.c_str(), color, 3);

        LidarData::Ptr data(new LidarData());
        data->mName = lidarNames[i];
        data->mChannel = channel;
        data->mActive = false;
        data->mColor = Eigen::Vector3f(color[0], color[1], color[2]);;
        data->mScans.set_capacity(1000);
        data->mProjector.reset(laser_projector_new(botParam, getBotFrames(),
                                                   channel.c_str(), 1),
                               laser_projector_destroy);
        mDataMap[data->mChannel] = data;
      }
      g_strfreev(lidarNames);
    }
  }

  ~LidarTestRenderer() {
  }

  void onLidar(const lcm::ReceiveBuffer* iBuf,
               const std::string& iChannel,
               const bot_core::planar_lidar_t* iMessage) {
    LidarDataMap::const_iterator item = mDataMap.find(iChannel);
    if (item == mDataMap.end()) return;
    LidarData::Ptr data = item->second;
    bot_core_planar_lidar_t msg;
    msg.utime = iMessage->utime;
    msg.nranges = iMessage->nranges;
    msg.nintensities = iMessage->nintensities;
    msg.rad0 = iMessage->rad0;
    msg.radstep = iMessage->radstep;
    msg.ranges = const_cast<float*>(&iMessage->ranges[0]);
    msg.intensities = const_cast<float*>(&iMessage->intensities[0]);
    laser_projected_scan* scan = laser_create_projected_scan_from_planar_lidar
      (data->mProjector.get(), &msg, bot_frames_get_root_name(getBotFrames()));
    if (scan == NULL) return;
    data->mScans.push_back
      (LidarData::ScanPtr(scan, laser_destroy_projected_scan));
  }

  void draw() {
    mPointBuffer.clear();

    // iterate over different laser sensors
    LidarDataMap::const_iterator dataIter = mDataMap.begin();
    for (; dataIter != mDataMap.end(); ++dataIter) {
      BotFrames* botFrames = getBotFrames();
      LidarData::Ptr data = dataIter->second;

      // iterate over scans from this sensor
      int counter = 0;
      LidarData::ScanBuffer::reverse_iterator scanIter = data->mScans.rbegin();
      for (; (scanIter != data->mScans.rend()) && (counter < mScanHistory);
           ++scanIter, ++counter) {
        LidarData::ScanPtr scan = *scanIter;

        // update scan pose if necessary
        if (scan->projection_status == 0) {
          laser_update_projected_scan
            (data->mProjector.get(), scan.get(),
             bot_frames_get_root_name(botFrames));
        }

        // grab all valid points from this scan
        mPointBuffer.reserve(mPointBuffer.size() + scan->npoints);
        for (int i = 0; i < scan->npoints; ++i) {
          if (scan->point_status[i] > laser_valid_projection) continue;
          Eigen::Vector3f pt(scan->points[i].x, scan->points[i].y,
                             scan->points[i].z);
          mPointBuffer.push_back(pt);
        }
      }

      // set up mesh renderer parameters
      mMeshRenderer->setColor(data->mColor[0], data->mColor[1],
                              data->mColor[2]);
      mMeshRenderer->setPointSize(mPointSize);
      mMeshRenderer->setColorMode((maps::MeshRenderer::ColorMode)mColorMode);
      mMeshRenderer->setScaleRange(mMinZ, mMaxZ);
      BotTrans trans;
      bot_frames_get_trans(getBotFrames(), "head", "local", &trans);
      Eigen::Vector3f pos(trans.trans_vec[0], trans.trans_vec[1],
                          trans.trans_vec[2]);
      mMeshRenderer->setRangeOrigin(pos);

      // draw points only
      if (mDrawMode == DrawModePoints) {
        mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModePoints);
        mMeshRenderer->setData(mPointBuffer, std::vector<Eigen::Vector3i>());
      }

      // draw surfels (wireframe)
      else if ((mDrawMode == DrawModeSurfelsWire) ||
               (mDrawMode == DrawModeSurfelsFilled)) {
        std::vector<Eigen::Vector3f> vertices;
        std::vector<Eigen::Vector3i> faces;
        computeSurfels(data, vertices, faces);
        mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModeWireframe);
        if (mDrawMode == DrawModeSurfelsFilled) {
          mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModeFilled);
        }
        mMeshRenderer->setData(vertices, faces);
      }

      // execute gl drawing
      mMeshRenderer->draw();
    }
  }


  void computeSurfels(const LidarData::Ptr& iData,
                      std::vector<Eigen::Vector3f>& oVertices,
                      std::vector<Eigen::Vector3i>& oFaces) {

    if (iData->mScans.size() == 0) return;

    // set up some constants
    const int r1(3), r2(3), w1(2*r1+1), w2(2*r2+1);
    const int numScans = iData->mScans.size();
    const int stopIndex = std::max(r1, numScans-mScanHistory);
    const float theta2 = iData->mScans[0]->rawScan->radstep;
    const int valid = laser_valid_projection;

    // compute all scan plane normals
    std::vector<Eigen::Vector3f> scanPlaneNormals(numScans);
    for (int i = 1; i < numScans-1; ++i) {
      BotTrans& trans = iData->mScans[i]->origin;
      Eigen::Quaternionf q(trans.rot_quat[0], trans.rot_quat[1],
                           trans.rot_quat[2], trans.rot_quat[3]);
      scanPlaneNormals[i] = q.matrix().col(2);
    }

    // allocate vectors
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(w1*w2);
    oVertices.clear();
    oFaces.clear();

    // iterate over scans (most to least recent)
    int cnt = 0;
    for (int i = numScans-1-r1; i >= stopIndex; --i) {

      // bump up allocation for output mesh arrays
      oVertices.reserve(oVertices.size() + 4*iData->mScans[i]->npoints);
      oFaces.reserve(oFaces.size() + 2*iData->mScans[i]->npoints);

      // get sensor origin for range computations
      BotTrans& trans = iData->mScans[i]->origin;
      Eigen::Vector3f scanOrigin(trans.trans_vec[0], trans.trans_vec[1],
                                 trans.trans_vec[2]);

      // get lidar look direction
      Eigen::Quaternionf q(trans.rot_quat[0], trans.rot_quat[1],
                           trans.rot_quat[2], trans.rot_quat[3]);
      Eigen::Vector3f xDir = q.matrix().col(0);

      // get inter-scan angle
      float anglePos = acos(scanPlaneNormals[i].dot(scanPlaneNormals[i+1]));
      float angleNeg = acos(scanPlaneNormals[i].dot(scanPlaneNormals[i-1]));
      float theta1 = 0.5*(anglePos + angleNeg);

      // iterate over points in this scan
      for (int j = r2; j < iData->mScans[i]->npoints-r2; ++j) {
        if (iData->mScans[i]->point_status[j] > valid) {
          continue;
        }

        // gather neighboring points for normal estimation
        pts.clear();
        for (int m = -r1; m <= r1; ++m) {
          for (int n = -r2; n <= r2; ++n) {
            if (iData->mScans[i+m]->point_status[j+n] > valid) continue;
            point3d_t& p = iData->mScans[i+m]->points[j+n];
            pts.push_back(Eigen::Vector3f(p.x, p.y, p.z));
          }
        }
        if (pts.size() < 3) continue;

        // estimate normal
        Eigen::Matrix3f pca = computePca(pts);
        Eigen::Vector3f normal = pca.col(2);

        // compute constrained principal directions
        Eigen::Vector3f v2 = normal.cross(scanPlaneNormals[i]);
        Eigen::Vector3f v1 = v2.cross(normal);

        // compute size in each non-normal direction
        point3d_t& p = iData->mScans[i]->points[j];
        Eigen::Vector3f pt(p.x, p.y, p.z);
        float size1(0.1), size2(0.1);
        float dist = (pt-scanOrigin).norm();
        if (mSurfelMode == SurfelModeFixed) {
          size1 = size2 = mSurfelScale/100;
        } else if (mSurfelMode == SurfelModeDistance) {
          size1 = size2 = dist*mSurfelScale/100;
        } else if (mSurfelMode == SurfelModeAngle) {
          Eigen::Vector3f xPt = scanOrigin + xDir*dist;
          float dist1 = (xPt-pt).norm();
          size1 = theta1*dist1*mSurfelScale/2;
          size2 = theta2*dist*mSurfelScale/2;
        } else if (mSurfelMode == SurfelModeNeighbors) {
          if ((iData->mScans[i]->point_status[j-1] > valid) ||
              (iData->mScans[i]->point_status[j+1] > valid) ||
              (iData->mScans[i-1]->point_status[j] > valid) ||
              (iData->mScans[i+1]->point_status[j] > valid)) continue;
          point3d_t p3d = iData->mScans[i]->points[j-1];
          Eigen::Vector3f ptIntra1(p3d.x, p3d.y, p3d.z);
          p3d = iData->mScans[i]->points[j+1];
          Eigen::Vector3f ptIntra2(p3d.x, p3d.y, p3d.z);
          p3d = iData->mScans[i-1]->points[j];
          Eigen::Vector3f ptInter1(p3d.x, p3d.y, p3d.z);
          p3d = iData->mScans[i+1]->points[j];
          Eigen::Vector3f ptInter2(p3d.x, p3d.y, p3d.z);
          size1 = (ptInter2-ptInter1).norm()/2*mSurfelScale;
          size2 = (ptIntra2-ptIntra1).norm()/2*mSurfelScale;
          if ((size1 > 1) || (size2 > 1)) continue;
        } else if (mSurfelMode == SurfelModeHallucinate) {
          // TODO
        }

        // scale principal vectors
        v1 *= size1/v1.norm();
        v2 *= size2/v2.norm();

        // create a quad (two triangles) for this point
        oVertices.push_back(pt-v1+v2);
        oVertices.push_back(pt+v1+v2);
        oVertices.push_back(pt+v1-v2);
        oVertices.push_back(pt-v1-v2);
        oFaces.push_back(Eigen::Vector3i(4*cnt,4*cnt+3,4*cnt+1));
        oFaces.push_back(Eigen::Vector3i(4*cnt+1,4*cnt+3,4*cnt+2));
        ++cnt;
      }
    }
  }

  Eigen::Matrix3f computePca(const std::vector<Eigen::Vector3f>& iPoints) {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    Eigen::Matrix3f meanSq = Eigen::Matrix3f::Zero();
    int n = iPoints.size();
    for (int i = 0; i < n; ++i) {
      mean += iPoints[i];
      meanSq += iPoints[i]*iPoints[i].transpose();
    }
    mean /= n;
    meanSq /= n;
    Eigen::Matrix3f cov = meanSq - mean*mean.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3f> svd;
    svd.compute(cov, Eigen::ComputeFullV);
    Eigen::Matrix3f v = svd.matrixV();
    Eigen::JacobiSVD<Eigen::Matrix3f>::SingularValuesType vals =
      svd.singularValues();
    for (int k = 0; k < 3; ++k) {
      v.col(k) *= sqrt(vals[k])/v.col(k).norm();
    }
    return v;
  }
};


// this is the single setup method exposed for integration with the viewer
void lidartest_renderer_setup(const std::string& iName,
                              BotViewer* iViewer, const int iPriority,
                              const lcm_t* iLcm,
                              const BotParam* iParam,
                              const BotFrames* iFrames) {
  new LidarTestRenderer(iName, iViewer, iPriority, iLcm, iParam, iFrames);
}
