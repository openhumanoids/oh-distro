#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include <bot_vis/bot_vis.h>
#include <bot_core/camtrans.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include <lcmtypes/drc/map_image_t.hpp>
#include <lcmtypes/bot_core/image_t.hpp>
#include <zlib.h>
#include <image_utils/jpeg.h>
#include <image_utils/pixels.h>

static const char* RENDERER_NAME = "Heightmap";

static const char* PARAM_COLOR_MODE = "Color Mode";
static const char* PARAM_COLOR_MODE_Z_MAX_Z = "Red Height";
static const char* PARAM_COLOR_MODE_Z_MIN_Z = "Blue Height";
static const char* PARAM_HEIGHT_MODE = "Height Mode";
static const char* PARAM_COLOR_ALPHA = "Alpha";
static const char* PARAM_NAME_FREEZE = "Freeze";

enum ColorMode {
  COLOR_MODE_Z,
  COLOR_MODE_ORANGE,
  COLOR_MODE_GRADIENT,
  COLOR_MODE_CAMERA
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
  BotParam* mBotParam;
  BotFrames* mBotFrames;
  BotGtkParamWidget* mWidget;

  drc::map_image_t mHeightMap;
  drc::map_image_t mCostMap;
  bot_core::image_t mCameraImage;

  HeightMode mHeightMode;
  ColorMode mColorMode;
  double mColorAlpha;
  double mMaxZ;
  double mMinZ;

  GLuint mVertexBufferId;
  GLuint mColorBufferId;
  GLuint mTexCoordBufferId;
  GLuint mIndexBufferId;
  GLuint mCameraTextureId;
  bool mFirstDraw;

  BotCamTrans* mCamTrans;

  RendererHeightMap() {
    mHeightMap.total_bytes = 0;
    mCostMap.total_bytes = 0;
    mCameraImage.size = 0;
    mCamTrans = NULL;
    mFirstDraw = true;
  }

  void onCameraImage(const lcm::ReceiveBuffer* iBuf,
                     const std::string& iChannel,
                     const bot_core::image_t* iMessage) {
    if (bot_gtk_param_widget_get_bool (mWidget, PARAM_NAME_FREEZE)) {
      return;
    }

    mCameraImage = *iMessage;
    if (iMessage->pixelformat == PIXEL_FORMAT_MJPEG) {
      int stride = iMessage->width * 3;
      mCameraImage.data.resize(iMessage->height * stride);
      mCameraImage.pixelformat = 0;
      jpeg_decompress_8u_rgb(&iMessage->data[0], iMessage->size,
                             &mCameraImage.data[0],
                             iMessage->width, iMessage->height, stride);
    }
    else {
      // TODO: this will break unless rgb3
    }

    if (mCamTrans == NULL) {
      mCamTrans = bot_param_get_new_camtrans(mBotParam, "CAMERALEFT");
    }

    bot_viewer_request_redraw(mViewer);
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
      int ret = uncompress(&bytes[0], &uncompressedSize,
                           &mapImage->data[0], mapImage->total_bytes);
      if (ret != Z_OK) {
        std::cout << "CANNOT UNCOMPRESS" << std::endl;
        std::cout << "RET=" << ret << std::endl;
      }
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
          uint8_t val = bytes[i*rowBytes+j];
          out[i*mapImage->width+j] =
            (val > 0) ? val : -std::numeric_limits<float>::max();
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

    // save state
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_ALL_ATTRIB_BITS);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();

    // set rendering flags
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_TEXTURE_2D);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); 
    glEnable (GL_RESCALE_NORMAL);
    if (self->mHeightMode == HEIGHT_MODE_MESH) {
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    else {
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    // transform from image to world
    Eigen::Matrix4d imageToLocal;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        imageToLocal(i,j) = img.transform[i][j];
      }
    }
    imageToLocal = imageToLocal.inverse();
    GLdouble imageToLocalGl[16];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        imageToLocalGl[4*j + i] = imageToLocal(i,j);
      }
    }
    glMatrixMode(GL_MODELVIEW);
    glMultMatrixd(imageToLocalGl);

    // assign all buffers
    if (self->mFirstDraw) {
      glGenBuffers(1, &self->mVertexBufferId);
      glGenBuffers(1, &self->mColorBufferId);
      glGenBuffers(1, &self->mTexCoordBufferId);
      glGenBuffers(1, &self->mIndexBufferId);
      glGenTextures(1, &self->mCameraTextureId);
      self->mFirstDraw = false;
    }
    
    // create vertex buffer
    float* data = (float*)(&img.data[0]);
    int numVertices = img.width*img.height;
    std::vector<float> vertexBuf(numVertices*3);
    for (int i = 0; i < img.height; ++i) {
      for (int j = 0; j < img.width; ++j) {
        int idx = i*img.width + j;
        float z = (self->mHeightMode == HEIGHT_MODE_NONE) ?
          self->mMinZ : data[idx];
        vertexBuf[idx*3+0] = j;
        vertexBuf[idx*3+1] = i;
        vertexBuf[idx*3+2] = z;
      }
    }
    glBindBuffer(GL_ARRAY_BUFFER, self->mVertexBufferId);
    glBufferData(GL_ARRAY_BUFFER, numVertices*3*sizeof(float),
                 &vertexBuf[0], GL_STATIC_DRAW);
    glEnableClientState(GL_VERTEX_ARRAY);
    //glVertexPointer(3, GL_FLOAT, 0, &vertexBuf[0]);
    glVertexPointer(3, GL_FLOAT, 0, 0);

    // create color buffer
    if (self->mColorMode != COLOR_MODE_CAMERA) {
      std::vector<uint8_t> colorBuf(numVertices*4);
      float orange[] = {1.0, 0.5, 0.0};
      float* outColor;
      for (int i = 0; i < img.height; ++i) {
        for (int j = 0; j < img.width; ++j) {
          int idx = i*img.width + j;
          outColor = orange;
          switch(self->mColorMode) {
          case COLOR_MODE_Z:
            {
              float z = data[idx]*imageToLocal(2,2) + imageToLocal(2,3);
              outColor = bot_color_util_jet((z - self->mMinZ) /
                                            (self->mMaxZ - self->mMinZ));
            }
            break;
          case COLOR_MODE_GRADIENT:
            if (self->mCostMap.total_bytes > 0) {
              outColor = bot_color_util_jet(self->mCostMap.data[idx]);
            }
            break;
          case COLOR_MODE_ORANGE:
          default:
            break;
          }
          colorBuf[idx*4+0] = outColor[0]*255;
          colorBuf[idx*4+1] = outColor[1]*255;
          colorBuf[idx*4+2] = outColor[2]*255;
          colorBuf[idx*4+3] = self->mColorAlpha*255;
        }        
      }
      glBindBuffer(GL_ARRAY_BUFFER, self->mColorBufferId);
      glBufferData(GL_ARRAY_BUFFER, numVertices*4,
                   &colorBuf[0], GL_STATIC_DRAW);
      //glColorPointer(4, GL_UNSIGNED_BYTE, 0, &colorBuf[0]);
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(4, GL_UNSIGNED_BYTE, 0, 0);
    }

    // create texture coordinate buffer
    else if (self->mCameraImage.size > 0) {
      int texCoordSize = 3;
      std::vector<float> texCoordBuf(numVertices*texCoordSize);
      for (int i = 0; i < img.height; ++i) {
        for (int j = 0; j < img.width; ++j) {
          int idx = i*img.width + j;
          float x = (float)j;
          float y = (float)i;
          float z = data[idx];
          texCoordBuf[idx*texCoordSize+0] = x;
          texCoordBuf[idx*texCoordSize+1] = y;
          texCoordBuf[idx*texCoordSize+2] = z;
        }
      }

      // texture coordinates transformation
      glMatrixMode(GL_TEXTURE);
      glLoadIdentity();
      {
        double width = bot_camtrans_get_width(self->mCamTrans);
        double height = bot_camtrans_get_height(self->mCamTrans);
        double K00 = bot_camtrans_get_focal_length_x(self->mCamTrans);
        double K11 = bot_camtrans_get_focal_length_y(self->mCamTrans);
        double K01 = bot_camtrans_get_skew(self->mCamTrans);
        double K02 = bot_camtrans_get_principal_x(self->mCamTrans);
        double K12 = bot_camtrans_get_principal_y(self->mCamTrans);

        double matxProject[16], matxProjectGl[16];
        memset(matxProject, 0, 16*sizeof(double));
        matxProject[0*4 + 0] = K00;
        matxProject[0*4 + 1] = K01;
        matxProject[0*4 + 2] = K02;
        matxProject[1*4 + 1] = K11;
        matxProject[1*4 + 2] = K12;
        matxProject[2*4 + 3] = 1;
        matxProject[3*4 + 2] = 1;
        bot_matrix_transpose_4x4d(matxProject, matxProjectGl);
        glScaled(1/width,1/height,1);
        glMultMatrixd(matxProjectGl);        
      }
      double matx[16], matxGl[16];
      bot_frames_get_trans_mat_4x4_with_utime(self->mBotFrames,
                                              "local", "CAMERA",
                                              self->mCameraImage.utime, matx);
      bot_matrix_transpose_4x4d(matx, matxGl);
      glMultMatrixd(matxGl);
      glMultMatrixd(imageToLocalGl);

      // draw texture
      glColor4ub(255,255,255,255*self->mColorAlpha);
      glBindTexture(GL_TEXTURE_2D, self->mCameraTextureId);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      glTexImage2D(GL_TEXTURE_2D, 0, 3, self->mCameraImage.width,
                   self->mCameraImage.height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                   &self->mCameraImage.data[0]);

      glBindBuffer(GL_ARRAY_BUFFER, self->mTexCoordBufferId);
      glBufferData(GL_ARRAY_BUFFER, numVertices*texCoordSize*sizeof(float),
                   &texCoordBuf[0], GL_STATIC_DRAW);
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);
      //glTexCoordPointer(2, GL_FLOAT, 0, &texCoordBuf[0]);
      glTexCoordPointer(texCoordSize, GL_FLOAT, 0, 0);
    }

    // determine valid triangles
    std::vector<uint32_t> indexBuf;
    for (int i = 0; i < img.height-1; i++) {
      for (int j = 0; j < img.width-1; j++) {
        int idx = i*img.width + j;
        double z00 = data[idx];
        double z10 = data[idx+1];
        double z01 = data[idx+img.width];
        double z11 = data[idx+img.width+1];
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
          pushIndices(indexBuf, idx, idx+1, idx+img.width);
          pushIndices(indexBuf, idx+1+img.width, idx+1, idx+img.width);
        }	  
        else {
          if (!valid00) {
            pushIndices(indexBuf, idx+1, idx+img.width, idx+1+img.width);
          }
          else if (!valid10) {
            pushIndices(indexBuf, idx, idx+img.width, idx+1+img.width);
          }
          else if (!valid01) {
            pushIndices(indexBuf, idx, idx+1+img.width, idx+1);
          }
          else if (!valid11) {
            pushIndices(indexBuf, idx, idx+img.width, idx+1);
          }
        }
      }
    }

    // create index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self->mIndexBufferId);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBuf.size()*sizeof(uint32_t),
                 &indexBuf[0], GL_STATIC_DRAW);
    /*
    glDrawElements(GL_TRIANGLES, indexBuf.size(),
                   GL_UNSIGNED_INT, &indexBuf[0]);
    */
    glDrawElements(GL_TRIANGLES, indexBuf.size(), GL_UNSIGNED_INT, 0);

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);

    // restore state
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glPopClientAttrib();
    glPopAttrib();
  }

  static void pushIndices(std::vector<uint32_t>& ioInds,
                          const int iA, const int iB, const int iC) {
    ioInds.push_back(iA);
    ioInds.push_back(iB);
    ioInds.push_back(iC);
  }

};


// TODO: move these inside

static void
heightmap_renderer_free (BotRenderer *renderer) {
  RendererHeightMap *self = (RendererHeightMap*) renderer;
  // TODO: free gl stuff
  if (self->mCamTrans != NULL) {
    bot_camtrans_destroy(self->mCamTrans);
  }
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
                                "Gradient", COLOR_MODE_GRADIENT,
                                "Orange", COLOR_MODE_ORANGE,
                                "Camera", COLOR_MODE_CAMERA, NULL);  

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
  self->mLcm->subscribe("CAMERALEFT", &RendererHeightMap::onCameraImage, self);

  std::cout << "Finished Setting Up Heightmap Renderer" << std::endl;

  bot_viewer_add_renderer(viewer, &self->mRenderer, priority);
}
