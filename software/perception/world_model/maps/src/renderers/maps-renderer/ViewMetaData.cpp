#include "ViewMetaData.hpp"

#include "MapsRenderer.hpp"
#include "MeshRenderer.hpp"

#include <lcmtypes/drc/data_request_t.hpp>
#include <lcmtypes/drc/map_snapshot_request_t.hpp>
#include <gtkmm-renderer/RendererBase.hpp>

#include <drc_utils/Clock.hpp>

#include <maps/ViewBase.hpp>
#include <maps/ViewClient.hpp>
#include <maps/Utils.hpp>

using namespace maps;

namespace {
  using namespace drc;

  struct Attributes {
    std::string mLabel;
    Eigen::Vector3f mColor;
    double mMinZ;
    double mMaxZ;
    double mPointSize;
    int mColorMode;
    int mMeshMode;

    Attributes() {
      mColor << 0,0,0;
      mMinZ = 0;
      mMaxZ = 1;
      mPointSize = 3;
      mColorMode = MeshRenderer::ColorModeHeight;
      mMeshMode = MeshRenderer::MeshModePoints;
    }
  };

  struct Globals {
    std::map<int64_t,Attributes> mAttributesMap;

    Globals() {
      add(data_request_t::OCTREE_SCENE, "Octree Scene", 0,0,0, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::HEIGHT_MAP_SCENE, "Height Scene", 1,0,0, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::HEIGHT_MAP_COARSE, "Height Coarse", 1,0.5,0, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::HEIGHT_MAP_DENSE, "Height Dense", 1,0.5,0, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::DEPTH_MAP_SCENE, "Depth Scene", 0.5,0,0.5, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::DEPTH_MAP_WORKSPACE, "Depth Workspace", 0,0,1, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::STEREO_MAP_HEAD, "Stereo Head", 0.1,1,0.1, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::STEREO_MAP_LHAND, "Stereo L.Hand", 0.1,0.7,0.1, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::STEREO_MAP_RHAND, "Stereo R.Hand", 0.1,0.4,0.1, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::DENSE_CLOUD_BOX, "Cloud Box", 1,0,1, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::DENSE_CLOUD_LHAND, "Cloud L.Hand", 1.0,0.5,0.3, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(data_request_t::DENSE_CLOUD_RHAND, "Cloud R.Hand", 1.0,0.5,0.3, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(1000, "Heightmap Controller", 0.5,0,0, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
      add(9999,"Debug", 0,0,0, 0,1,3,
          MeshRenderer::ColorModeHeight, MeshRenderer::MeshModePoints);
    }

    void add(const int64_t iId, const std::string& iLabel,
             const double iR, const double iG, const double iB,
             const double iMinZ, const double iMaxZ,
             const double iPointSize, const int iColorMode,
             const int iMeshMode) {
      Attributes attributes;
      attributes.mLabel = iLabel;
      attributes.mColor << iR,iG,iB;
      attributes.mMinZ = iMinZ;
      attributes.mMaxZ = iMaxZ;
      attributes.mPointSize = iPointSize;
      attributes.mColorMode = iColorMode;
      attributes.mMeshMode = iMeshMode;
      mAttributesMap[iId] = attributes;
    }
  };

  Globals gGlobals;
}

struct ViewMetaData::Helper {
  struct Frustum {
    std::vector<Eigen::Vector4f> mPlanes;
    float mNear;
    float mFar;
    Eigen::Vector3f mPos;
    Eigen::Vector3f mDir;
  };

  MapsRenderer* mRenderer;
  int64_t mViewId;
  bool mVisible;

  Attributes mAttributes;

  Gtk::Box* mBox;
  Gtk::ToggleButton* mToggleButton;
  Eigen::Isometry3f mLatestTransform;
  
  Helper(const MapsRenderer* iRenderer, const int64_t iViewId) {
    mRenderer = (MapsRenderer*)iRenderer;
    mBox = NULL;

    mViewId = iViewId;
    auto item = gGlobals.mAttributesMap.find(mViewId);
    if (item != gGlobals.mAttributesMap.end()) {
      mAttributes = item->second;
    }
    else {
      mAttributes.mLabel = static_cast<std::ostringstream*>
        (&(std::ostringstream() << mViewId) )->str();
      mAttributes.mColor << (double)rand()/RAND_MAX,
        (double)rand()/RAND_MAX, (double)rand()/RAND_MAX;
    }
    mVisible = true;
    mLatestTransform = Eigen::Isometry3f::Identity();
  }

  /*
  void drawFrustum(const Frustum& iFrustum, const Eigen::Vector3f& iColor) {
    std::vector<Eigen::Vector3f> vertices;
    std::vector<std::vector<int> > faces;
    maps::Utils::polyhedronFromPlanes(iFrustum.mPlanes, vertices, faces);
    glColor3f(iColor[0], iColor[1], iColor[2]);
    glLineWidth(3);
    for (size_t i = 0; i < faces.size(); ++i) {
      glBegin(GL_LINE_LOOP);
      for (size_t j = 0; j < faces[i].size(); ++j) {
        Eigen::Vector3f pt = vertices[faces[i][j]];
        glVertex3f(pt[0], pt[1], pt[2]);
      }
      glEnd();
    }
  }
  */

  void draw(const std::shared_ptr<maps::ViewBase>& iView,
            const std::shared_ptr<MeshRenderer>& iMeshRenderer) {
    if (!mVisible) return;

    // set mesh properties
    iMeshRenderer->setRangeOrigin(mLatestTransform.translation());
    iMeshRenderer->setScaleRange(mAttributes.mMinZ, mAttributes.mMaxZ);
    iMeshRenderer->setPointSize(mAttributes.mPointSize);
    Eigen::Projective3f worldToMap = iView->getTransform();
    Eigen::Projective3f mapToWorld = worldToMap.inverse();
    Eigen::Matrix3f calib;
    Eigen::Isometry3f pose;
    bool ortho;
    maps::Utils::factorViewMatrix(worldToMap, calib, pose, ortho);
    iMeshRenderer->setNormalZero(-pose.linear().col(2));

    /*
    // draw frustum
    ViewBase::Spec viewSpec;
    if (mViewClient.getSpec(id, viewSpec)) {
      if (viewSpec.mClipPlanes.size() > 0) {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        if (viewSpec.mRelativeLocation) {
          glMultMatrixf(data->mLatestTransform.data());
        }
        Frustum frustum;
        frustum.mPlanes = viewSpec.mClipPlanes;
        drawFrustum(frustum, data->mColor);
        glPopMatrix();
      }
    }
    */

    // see whether we need to (and can) get a mesh representation
    bool usePoints = false;
    maps::TriangleMesh::Ptr mesh;
    if (mAttributes.mMeshMode == MeshRenderer::MeshModePoints) {
      usePoints = true;
    }
    else {
      mesh = iView->getAsMesh(false);
      if (mesh == NULL) usePoints = true;
    }

    // just a point cloud
    if (usePoints) {
      mesh.reset(new maps::TriangleMesh());
      maps::PointCloud::Ptr cloud = iView->getAsPointCloud(false);
      mesh->mVertices.reserve(cloud->size());
      for (size_t i = 0; i < cloud->size(); ++i) {
        mesh->mVertices.push_back((*cloud)[i].getVector3fMap());
      }
    }

    // set up mesh renderer
    iMeshRenderer->setColor(mAttributes.mColor[0],
                            mAttributes.mColor[1],
                            mAttributes.mColor[2]);
    iMeshRenderer->setColorMode
      ((MeshRenderer::ColorMode)mAttributes.mColorMode);
    iMeshRenderer->setMeshMode
      ((MeshRenderer::MeshMode)mAttributes.mMeshMode);
    if (usePoints) iMeshRenderer->setMeshMode(MeshRenderer::MeshModePoints);
    iMeshRenderer->setData(mesh->mVertices, mesh->mNormals,
                           mesh->mFaces, mapToWorld);

    // draw this view's data
    iMeshRenderer->draw();
  }

  bool addWidgets(Gtk::Box* iBox) {
    if (mBox != NULL) return false;

    mBox = Gtk::manage(new Gtk::VBox());

    Gtk::Box* box = Gtk::manage(new Gtk::HBox());

    mToggleButton = Gtk::manage(new Gtk::ToggleButton("                    "));
    Gdk::Color color;
    color.set_rgb_p(mAttributes.mColor[0], mAttributes.mColor[1],
                    mAttributes.mColor[2]);
    mToggleButton->modify_bg(Gtk::STATE_ACTIVE, color);
    color.set_rgb_p((mAttributes.mColor[0]+1)/2, (mAttributes.mColor[1]+1)/2,
                    (mAttributes.mColor[2]+1)/2);
    mToggleButton->modify_bg(Gtk::STATE_PRELIGHT, color); 
    color.set_rgb_p(0.8, 0.8, 0.8);
    mToggleButton->modify_bg(Gtk::STATE_NORMAL, color);
    mToggleButton->signal_toggled().connect
      (sigc::mem_fun(*this, &Helper::onToggleButton));
    mToggleButton->set_active(true);
    box->pack_start(*(mToggleButton), false, false);
    Gtk::Label* label = Gtk::manage(new Gtk::Label(mAttributes.mLabel));
    box->pack_start(*label, false, false);
    if (mViewId != 1) {
      Gtk::Button* cancelButton = Gtk::manage(new Gtk::Button("X"));
      box->pack_start(*cancelButton, false, false);
      cancelButton->signal_clicked().connect
        (sigc::mem_fun(*this, &Helper::onCancelButton));
    }
    Gtk::Button* button = Gtk::manage(new Gtk::Button("save"));
    button->signal_clicked().connect
      (sigc::mem_fun(*this, &Helper::onSaveButton));
    box->pack_start(*button, false, false);

    mBox->pack_start(*box, false, false);

    box = Gtk::manage(new Gtk::HBox());
    std::vector<int> ids;
    std::vector<std::string> labels;
    ids = { MeshRenderer::ColorModeFlat, MeshRenderer::ColorModeHeight,
            MeshRenderer::ColorModeRange, MeshRenderer::ColorModeNormal,
            MeshRenderer::ColorModeCamera };
    labels = { "Flat", "Height", "Range", "Normal", "Camera" };
    Gtk::ComboBox* combo = 
      gtkmm::RendererBase::createCombo(mAttributes.mColorMode, labels, ids);
    combo->signal_changed().connect([mRenderer]{mRenderer->requestDraw();});
    box->pack_start(*combo, false, false);

    ids = { MeshRenderer::MeshModePoints, MeshRenderer::MeshModeWireframe,
            MeshRenderer::MeshModeFilled };
    labels = { "Points", "Wireframe", "Filled" };
    combo =
      gtkmm::RendererBase::createCombo(mAttributes.mMeshMode, labels, ids);
    combo->signal_changed().connect([mRenderer]{mRenderer->requestDraw();});
    box->pack_start(*combo, false, false);
    
    Gtk::HScale* slider =
      gtkmm::RendererBase::createSlider(mAttributes.mPointSize, 0.1, 10, 0.1);
    slider->signal_value_changed().connect([mRenderer]{mRenderer->requestDraw();});
    box->pack_start(*slider, true, true);

    mBox->pack_start(*box, false, false);

    box = Gtk::manage(new Gtk::HBox());
    label = Gtk::manage(new Gtk::Label("Z Rng"));
    box->pack_start(*label,false,false);
    slider = gtkmm::RendererBase::createSlider(mAttributes.mMinZ, -1, 2, 0.01);
    slider->signal_value_changed().connect([mRenderer]{mRenderer->requestDraw();});
    box->pack_start(*slider, true, true);
    slider = gtkmm::RendererBase::createSlider(mAttributes.mMaxZ, -1, 2, 0.01);
    slider->signal_value_changed().connect([mRenderer]{mRenderer->requestDraw();});
    box->pack_start(*slider, true, true);

    mBox->pack_start(*box, false, false);

    mBox->show_all();
    iBox->pack_start(*mBox, false, false);
    return true;
  }

  void onCancelButton() {
    maps::ViewBase::Spec spec;
    spec.mMapId = 0;
    spec.mViewId = mViewId;
    spec.mResolution = spec.mFrequency = 0;
    spec.mTimeMin = spec.mTimeMax = 0;
    spec.mActive = false;
    spec.mType = maps::ViewBase::TypePointCloud;
    mRenderer->mViewClient.request(spec);
    mRenderer->requestDraw();
  }

  void onSaveButton() {
    drc::map_snapshot_request_t msg;
    msg.utime = drc::Clock::instance()->getCurrentTime();
    msg.view_id = mViewId;
    msg.new_view_id = mViewId + 10000;
    msg.command = drc::map_snapshot_request_t::STORE;
    mRenderer->getLcm()->publish("MAP_SNAPSHOT_REQUEST", &msg);
  }

  void onToggleButton() {
    mVisible = mToggleButton->get_active();
    mRenderer->requestDraw();
  }

  void setLatestTransform(const Eigen::Isometry3f& iTransform) {
    mLatestTransform = iTransform;
  }
};




ViewMetaData::
ViewMetaData(const MapsRenderer* iRenderer, const int64_t iViewId) {
  mHelper.reset(new Helper(iRenderer, iViewId));
}

bool ViewMetaData::
addWidgets(Gtk::Box* iBox) {
  return mHelper->addWidgets(iBox);
}

void ViewMetaData::
draw(const std::shared_ptr<maps::ViewBase>& iView,
     const std::shared_ptr<MeshRenderer>& iMeshRenderer) {
  return mHelper->draw(iView, iMeshRenderer);
}

void ViewMetaData::
setLatestTransform(const Eigen::Isometry3f& iTransform) {
  return mHelper->setLatestTransform(iTransform);
}
