#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>

#include <lcm/lcm-cpp.hpp>
#include <bot_vis/bot_vis.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include <lcmtypes/drc/map_image_t.hpp>
#include <zlib.h>

static const char* RENDERER_NAME = "Heightmap";

static const char* PARAM_COLOR_MODE = "Color Mode";
static const char* PARAM_COLOR_MODE_Z_MAX_Z = "Red Height";
static const char* PARAM_COLOR_MODE_Z_MIN_Z = "Blue Height";
static const char* PARAM_HEIGHT_MODE = "Height Mode";
static const char* PARAM_COLOR_ALPHA = "Alpha";
static const char* PARAM_NAME_FREEZE = "Freeze";

enum ColorMode {
  COLOR_MODE_RANGE,
  COLOR_MODE_Z,
  COLOR_MODE_ORANGE,
  COLOR_MODE_GRADIENT
};

enum HeightMode {
  HEIGHT_MODE_MESH,
  HEIGHT_MODE_WIRE,
  HEIGHT_MODE_NONE,
};


struct RendererHeightMap {

  BotRenderer mRenderer;
  BotViewer* mViewer;
  boost::shared_ptr<lcm::LCM> mLcm;
  BotGtkParamWidget* mWidget;

  drc::map_image_t mHeightMap;
  drc::map_image_t mCostMap;

  HeightMode mHeightMode;
  ColorMode mColorMode;
  double mColorAlpha;
  double mMaxZ;
  double mMinZ;

  RendererHeightMap() {
    mHeightMap.total_bytes = 0;
    mCostMap.total_bytes = 0;
  }

  void onMapImage(const lcm::ReceiveBuffer* iBuf,
                  const std::string& iChannel,
                  const drc::map_image_t* iMessage) {
    if (bot_gtk_param_widget_get_bool (mWidget, PARAM_NAME_FREEZE)) {
      return;
    }

    drc::map_image_t* mapImage = NULL;

    if (!iChannel.compare("HEIGHT_MAP")) {
      mapImage = &mHeightMap;
    }
    else if (!iChannel.compare("COST_MAP")) {
      mapImage = &mCostMap;
    }
    else {
      return;
    }

    *mapImage = *iMessage;

    std::vector<uint8_t> bytes;

    // uncompress if necessary
    if (mapImage->compression == drc::map_image_t::COMPRESSION_ZLIB) {
      bytes.resize(mapImage->row_bytes*mapImage->height);
      unsigned long uncompressedSize;
      uncompress(&bytes[0], &uncompressedSize,
                 &mapImage->data[0], mapImage->total_bytes);
      mapImage->compression = drc::map_image_t::COMPRESSION_NONE;
    }
    else {
      bytes = mapImage->data;
    }

    // convert from uint8 if necessary
    if (mapImage->format == drc::map_image_t::FORMAT_GRAY_UINT8) {
      int rowBytes = mapImage->row_bytes;
      mapImage->row_bytes = sizeof(float)*mapImage->width;
      mapImage->total_bytes = mapImage->row_bytes*mapImage->height;
      mapImage->format = drc::map_image_t::FORMAT_GRAY_FLOAT32;
      mapImage->data.resize(mapImage->total_bytes);
      float* out = (float*)(&mapImage->data[0]);
      for (int i = 0; i < mapImage->height; ++i) {
        for (int j = 0; j < mapImage->width; ++j) {
          out[i*mapImage->width+j] = mapImage->data[i*rowBytes+j];
        }
      }
    }
    else {
      mapImage->data = bytes;
    }

    bot_viewer_request_redraw(mViewer);
  }

  static void onParamWidgetChanged(BotGtkParamWidget* iWidget,
                                   const char *iName, 
                                   RendererHeightMap *self) {
    self->mMaxZ =
      bot_gtk_param_widget_get_double(iWidget, PARAM_COLOR_MODE_Z_MAX_Z);
    self->mMinZ =
      bot_gtk_param_widget_get_double(iWidget, PARAM_COLOR_MODE_Z_MIN_Z);
    self->mHeightMode = (HeightMode)
      bot_gtk_param_widget_get_enum(iWidget, PARAM_HEIGHT_MODE);
    self->mColorMode = (ColorMode)
      bot_gtk_param_widget_get_enum(iWidget, PARAM_COLOR_MODE);
    self->mColorAlpha =
      bot_gtk_param_widget_get_double(iWidget, PARAM_COLOR_ALPHA);
  
    bot_viewer_request_redraw(self->mViewer);
  }

  static void draw(BotViewer *iViewer, BotRenderer *iRenderer) {
    RendererHeightMap *self = (RendererHeightMap*) iRenderer->user;
    drc::map_image_t& img = self->mHeightMap;
    if (img.total_bytes == 0) {
      return;
    }
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
    glEnable (GL_RESCALE_NORMAL);

    // modify transform from image to world
    glPushMatrix();
    Eigen::Matrix4d xform;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        xform(i,j) = img.transform[i][j];
      }
    }
    xform = xform.inverse();
    GLdouble matx[16];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        matx[4*j + i] = xform(i,j);
      }
    }
    glMultMatrixd(matx);

    // draw triangles
    // TODO: use vertex buffer for efficiency
    float* data = (float*)(&img.data[0]);
    for (int i=0; i < img.height-1; i++) {
      for (int j=0; j < img.width-1; j++) {
        int index = i*img.width + j;
        double z00 = data[index];
        double z10 = data[index+1];
        double z01 = data[index+img.width];
        double z11 = data[index+img.width+1];
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
          self->drawTriangle(p00, p10, p01, index);
          self->drawTriangle(p11, p10, p01, index);
        }	  
        else {
          if (!valid00) {
            self->drawTriangle(p10, p01, p11, index);
          }
          else if (!valid10) {
            self->drawTriangle(p00, p01, p11, index);
          }
          else if (!valid01) {
            self->drawTriangle(p00, p11, p10, index);
          }
          else if (!valid11) {
            self->drawTriangle(p00, p01, p10, index);
          }
        }
      }
    }  

    glPopMatrix();
  }

  void drawTriangle(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c,
                    int index){
    double normVal;
    float *outColor; //rgb colour out
    if (mColorMode == COLOR_MODE_Z) {
      // normalized 0->1 height
      normVal = (a(2) - mMinZ) / (mMaxZ - mMinZ);
      outColor = bot_color_util_jet(normVal);
    } else if(mColorMode == COLOR_MODE_RANGE) {
      normVal = a.norm()/5;
      outColor = bot_color_util_jet(normVal);
    } else if(mColorMode == COLOR_MODE_ORANGE) {
      float orange[] = {1.0, 0.5, 0.0};
      outColor = orange;
    } else if(mColorMode == COLOR_MODE_GRADIENT) {
      if (mCostMap.total_bytes == 0) {
        float orange[] = {1.0, 0.5, 0.0};
        outColor = orange;
      } else {
        normVal = mCostMap.data[index]; // TODO: convert to float
        // TODO: index does not need to be passed in?
        outColor = bot_color_util_jet(normVal);
      }
    } else {
      normVal = 0.8;
      outColor = bot_color_util_jet(normVal);
    }
  
    if (mHeightMode == HEIGHT_MODE_MESH) {
      glBegin(GL_TRIANGLES);
      glColor4f(outColor[0], outColor[1], outColor[2], mColorAlpha);
      glVertex3f( a(0), a(1), a(2));
      glVertex3f( b(0), b(1), b(2));
      glVertex3f( c(0), c(1), c(2));
      glEnd();
    } else if(mHeightMode == HEIGHT_MODE_WIRE) {
      glBegin(GL_LINE_LOOP);
      glColor4f(outColor[0], outColor[1], outColor[2], mColorAlpha);
      glVertex3f( a(0), a(1), a(2));
      glVertex3f( b(0), b(1), b(2));
      glVertex3f( c(0), c(1), c(2));
      glEnd();
    } else if(mHeightMode == HEIGHT_MODE_NONE) {
      glBegin(GL_TRIANGLES);
      glColor4f(outColor[0], outColor[1], outColor[2], mColorAlpha);
      glVertex3f( a(0), a(1), mMinZ);
      glVertex3f( b(0), b(1), mMinZ);
      glVertex3f( c(0), c(1), mMinZ);
      glEnd();
    }
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
                                 lcm_t* lcm) {
  RendererHeightMap *self = new RendererHeightMap();
  self->mViewer = viewer;
  self->mRenderer.draw = RendererHeightMap::draw;
  self->mRenderer.destroy = heightmap_renderer_free;
  self->mRenderer.name = strdup("Heightmap");
  self->mRenderer.user = self;
  self->mRenderer.enabled = 1;
  self->mRenderer.widget = gtk_alignment_new (0, 0.5, 1.0, 0);
  self->mLcm.reset(new lcm::LCM(lcm));

  self->mWidget = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new ());
  gtk_container_add (GTK_CONTAINER (self->mRenderer.widget),
                     GTK_WIDGET(self->mWidget));
  gtk_widget_show (GTK_WIDGET (self->mWidget));

  bot_gtk_param_widget_add_double(self->mWidget, PARAM_COLOR_MODE_Z_MAX_Z,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  -20, 20.0, .1, 20.0 );
  bot_gtk_param_widget_add_double(self->mWidget, PARAM_COLOR_MODE_Z_MIN_Z,
                                  BOT_GTK_PARAM_WIDGET_SPINBOX,
                                  -20, 20.0, .1, -20.0 );

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_COLOR_MODE,
                                BOT_GTK_PARAM_WIDGET_MENU, COLOR_MODE_Z, 
                                "Height", COLOR_MODE_Z,
                                "Range", COLOR_MODE_RANGE, 
                                "Gradient", COLOR_MODE_GRADIENT,
                                "Orange", COLOR_MODE_ORANGE, NULL);  

  bot_gtk_param_widget_add_enum(self->mWidget, PARAM_HEIGHT_MODE,
                                BOT_GTK_PARAM_WIDGET_MENU, HEIGHT_MODE_MESH,
                                "Mesh", HEIGHT_MODE_MESH,
                                "Wire", HEIGHT_MODE_WIRE,
                                "None", HEIGHT_MODE_NONE, NULL);  
  
  
  bot_gtk_param_widget_add_double (self->mWidget, PARAM_COLOR_ALPHA,
                                   BOT_GTK_PARAM_WIDGET_SLIDER,
                                   0, 1, 0.001, 1);
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
