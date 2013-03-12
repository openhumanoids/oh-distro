#include "renderer_heightmap.hpp"

#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include <bot_vis/bot_vis.h>
#include <boost/shared_ptr.hpp>

#include <lcmtypes/drc/map_image_t.hpp>
#include <maps/RangeImageView.hpp>
#include <maps/LcmTranslator.hpp>
#include <pcl/range_image/range_image.h>

#include "MeshRenderer.hpp"

static const char* RENDERER_NAME = "Heightmap";

static const char* PARAM_COLOR_MODE = "Color Mode";
static const char* PARAM_COLOR_MODE_Z_MAX_Z = "Red Z";
static const char* PARAM_COLOR_MODE_Z_MIN_Z = "Blue Z";
static const char* PARAM_HEIGHT_MODE = "Height Mode";
static const char* PARAM_COLOR_ALPHA = "Alpha";
static const char* PARAM_POINT_SIZE = "Point Size";
static const char* PARAM_NAME_FREEZE = "Freeze";

enum ColorMode {
  COLOR_MODE_Z,
  COLOR_MODE_RANGE,
  COLOR_MODE_ORANGE,
  COLOR_MODE_GRADIENT,
  COLOR_MODE_CAMERA
};

enum HeightMode {
  HEIGHT_MODE_MESH,
  HEIGHT_MODE_WIRE,
  HEIGHT_MODE_POINTS,
  HEIGHT_MODE_NONE,
};

struct RendererHeightMap {

  BotRenderer mRenderer;
  BotViewer* mViewer;
  boost::shared_ptr<lcm::LCM> mLcm;
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  BotGtkParamWidget* mWidget;

  boost::shared_ptr<maps::MeshRenderer> mMeshRenderer;

  maps::RangeImageView mHeightMap;
  maps::RangeImageView mCostMap;
  HeightMode mHeightMode;

  RendererHeightMap() {
    mHeightMap.setId(-1);
    mCostMap.setId(-1);
  }

  void onMapImage(const lcm::ReceiveBuffer* iBuf,
                  const std::string& iChannel,
                  const drc::map_image_t* iMessage) {
    if (bot_gtk_param_widget_get_bool (mWidget, PARAM_NAME_FREEZE)) {
      return;
    }

    maps::RangeImageView* mapImage = NULL;
    if (!iChannel.compare("HEIGHT_MAP")) mapImage = &mHeightMap;
    else if (!iChannel.compare("COST_MAP")) mapImage = &mCostMap;
    else return;

    maps::LcmTranslator::fromLcm(*iMessage, *mapImage);
    bot_viewer_request_redraw(mViewer);
  }

  static void onParamWidgetChanged(BotGtkParamWidget* iWidget,
                                   const char *iName, 
                                   RendererHeightMap *self) {
    double val, val2;
    val = bot_gtk_param_widget_get_double(iWidget, PARAM_COLOR_MODE_Z_MIN_Z);
    val2 = bot_gtk_param_widget_get_double(iWidget, PARAM_COLOR_MODE_Z_MAX_Z);
    self->mMeshRenderer->setScaleRange(val,val2);
    val = bot_gtk_param_widget_get_double(iWidget, PARAM_COLOR_ALPHA);
    self->mMeshRenderer->setColorAlpha(val);
    val = bot_gtk_param_widget_get_double(iWidget, PARAM_POINT_SIZE);
    self->mMeshRenderer->setPointSize(val);
    int mode;
    mode = bot_gtk_param_widget_get_enum(iWidget, PARAM_HEIGHT_MODE);
    self->mHeightMode = HeightMode(mode);
    switch(self->mHeightMode) {
    case HEIGHT_MODE_WIRE:
      self->mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModeWireframe);
      break;
    case HEIGHT_MODE_MESH:
      self->mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModeFilled);
      break;
    case HEIGHT_MODE_POINTS:
      self->mMeshRenderer->setMeshMode(maps::MeshRenderer::MeshModePoints);
      break;
    default:
      break;
    }
    mode = bot_gtk_param_widget_get_enum(iWidget, PARAM_COLOR_MODE);
    switch(ColorMode(mode)) {
    case COLOR_MODE_ORANGE:
      self->mMeshRenderer->setColorMode(maps::MeshRenderer::ColorModeFlat);
      break;
    case COLOR_MODE_Z:
      self->mMeshRenderer->setColorMode(maps::MeshRenderer::ColorModeHeight);
      break;
    case COLOR_MODE_RANGE:
      {
        BotTrans trans;
        bot_frames_get_trans(self->mBotFrames, "head", "local", &trans);
        Eigen::Vector3f pos(trans.trans_vec[0], trans.trans_vec[1],
                            trans.trans_vec[2]);
        self->mMeshRenderer->setRangeOrigin(pos);
        self->mMeshRenderer->setColorMode(maps::MeshRenderer::ColorModeRange);
      }
      break;
    case COLOR_MODE_CAMERA:
      self->mMeshRenderer->setColorMode(maps::MeshRenderer::ColorModeCamera);
      break;
    default:
      break;
    }
  
    bot_viewer_request_redraw(self->mViewer);
  }

  static void draw(BotViewer *iViewer, BotRenderer *iRenderer) {
    RendererHeightMap *self = (RendererHeightMap*) iRenderer->user;
    if (self->mHeightMap.getId() < 0) {
      return;
    }

    pcl::RangeImage::Ptr img = self->mHeightMap.getRangeImage();

    // create vertex buffer
    float* data = img->getRangesArray();
    int width(img->width), height(img->height);
    int numVertices = width*height;
    std::vector<Eigen::Vector3f> vertices(numVertices);
    float minZ(1e10);
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        int idx = i*width + j;
        vertices[idx] = Eigen::Vector3f(j,i,data[idx]);
        minZ = std::min(minZ, data[idx]);
      }
    }
    if (self->mHeightMode == HEIGHT_MODE_NONE) {
      for (size_t i = 0; i < vertices.size(); ++i) {
        vertices[i][2] = minZ;
      }
    }

    // determine valid triangles
    std::vector<Eigen::Vector3i> faces;
    faces.reserve(2*numVertices);
    for (int i = 0; i < height-1; i++) {
      for (int j = 0; j < width-1; j++) {
        int idx = i*width + j;
        double z00 = data[idx];
        double z10 = data[idx+1];
        double z01 = data[idx+width];
        double z11 = data[idx+width+1];
        bool valid00 = z00 > -1e10;
        bool valid10 = z10 > -1e10;
        bool valid01 = z01 > -1e10;
        bool valid11 = z11 > -1e10;
        int validSum = (int)valid00 + (int)valid10 +
          (int)valid01 + (int)valid11;
        if (validSum < 3) {
          continue;
        }

        Eigen::Vector3f p00(j,i,z00);
        Eigen::Vector3f p10(j+1,i,z10);
        Eigen::Vector3f p01(j,i+1,z01);
        Eigen::Vector3f p11(j+1,i+1,z11);
	  
        if (validSum == 4) {
          faces.push_back(Eigen::Vector3i(idx, idx+1, idx+width));
          faces.push_back(Eigen::Vector3i(idx+1+width, idx+1, idx+width));
        }	  
        else {
          if (!valid00) {
            faces.push_back(Eigen::Vector3i(idx+1, idx+width, idx+1+width));
          }
          else if (!valid10) {
            faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1+width));
          }
          else if (!valid01) {
            faces.push_back(Eigen::Vector3i(idx, idx+1+width, idx+1));
          }
          else if (!valid11) {
            faces.push_back(Eigen::Vector3i(idx, idx+width, idx+1));
          }
        }
      }
    }

    // transform and draw
    Eigen::Projective3f meshToLocal =
      self->mHeightMap.getTransform().inverse();
    self->mMeshRenderer->setData(vertices, faces, meshToLocal);
    self->mMeshRenderer->draw();
  }

};


// TODO: move these inside

static void
heightmap_renderer_free (BotRenderer *renderer) {
  RendererHeightMap *self = (RendererHeightMap*) renderer;
  free (self->mRenderer.name);
  delete self;
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data) {
  RendererHeightMap *self = (RendererHeightMap*) user_data;
  bot_gtk_param_widget_load_from_key_file (self->mWidget, keyfile,
                                           RENDERER_NAME);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data) {
    RendererHeightMap *self = (RendererHeightMap*)user_data;
    bot_gtk_param_widget_save_to_key_file (self->mWidget, keyfile,
                                           RENDERER_NAME);
}

void
heightmap_add_renderer_to_viewer(BotViewer* viewer,
                                 int priority,
                                 lcm_t* lcm,
                                 BotParam* param,
                                 BotFrames* frames) {
  RendererHeightMap *self = new RendererHeightMap();
  self->mViewer = viewer;
  self->mRenderer.draw = RendererHeightMap::draw;
  self->mRenderer.destroy = heightmap_renderer_free;
  self->mRenderer.name = strdup("Heightmap");
  self->mRenderer.user = self;
  self->mRenderer.enabled = 1;
  self->mRenderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
  self->mLcm.reset(new lcm::LCM(lcm));
  self->mBotParam = param;
  self->mBotFrames = frames;
  self->mMeshRenderer.reset(new maps::MeshRenderer());
  self->mMeshRenderer->setBotParam(param);
  self->mMeshRenderer->setLcm(self->mLcm);
  self->mMeshRenderer->setColor(1,0.5,0);

  self->mWidget = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  gtk_container_add (GTK_CONTAINER (self->mRenderer.widget),
                     GTK_WIDGET(self->mWidget));
  gtk_widget_show (GTK_WIDGET (self->mWidget));

  bot_gtk_param_widget_add_double(self->mWidget, PARAM_COLOR_MODE_Z_MIN_Z,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  0, 1, 0.01, 0);
  bot_gtk_param_widget_add_double(self->mWidget, PARAM_COLOR_MODE_Z_MAX_Z,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  0, 1, 0.01, 1);

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_COLOR_MODE,
                                BOT_GTK_PARAM_WIDGET_MENU, COLOR_MODE_Z, 
                                "Height", COLOR_MODE_Z,
                                "Gradient", COLOR_MODE_GRADIENT,
                                "Orange", COLOR_MODE_ORANGE,
                                "Camera", COLOR_MODE_CAMERA, NULL);  

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_HEIGHT_MODE,
                                BOT_GTK_PARAM_WIDGET_MENU, HEIGHT_MODE_MESH,
                                "Filled", HEIGHT_MODE_MESH,
                                "Wireframe", HEIGHT_MODE_WIRE,
                                "Points", HEIGHT_MODE_POINTS,
                                "None", HEIGHT_MODE_NONE, NULL);  
  
  
  bot_gtk_param_widget_add_double (self->mWidget, PARAM_COLOR_ALPHA,
                                   BOT_GTK_PARAM_WIDGET_SLIDER,
                                   0, 1, 0.01, 1);
  bot_gtk_param_widget_add_double (self->mWidget, PARAM_POINT_SIZE,
                                   BOT_GTK_PARAM_WIDGET_SLIDER,
                                   0, 10, 0.1, 3);
  bot_gtk_param_widget_add_booleans (self->mWidget,
                                     BOT_GTK_PARAM_WIDGET_TOGGLE_BUTTON,
                                     PARAM_NAME_FREEZE, 0, NULL);
  
  g_signal_connect (G_OBJECT (self->mWidget), "changed",
                    G_CALLBACK (RendererHeightMap::onParamWidgetChanged),
                    self);


  // save widget modes:
  g_signal_connect (G_OBJECT (viewer), "load-preferences",
                    G_CALLBACK (on_load_preferences), self);
  g_signal_connect (G_OBJECT (viewer), "save-preferences",
                    G_CALLBACK (on_save_preferences), self);

  self->mLcm->subscribe("COST_MAP", &RendererHeightMap::onMapImage, self);
  self->mLcm->subscribe("HEIGHT_MAP", &RendererHeightMap::onMapImage, self);

  std::cout << "Finished Setting Up Heightmap Renderer" << std::endl;

  bot_viewer_add_renderer(viewer, &self->mRenderer, priority);
}
