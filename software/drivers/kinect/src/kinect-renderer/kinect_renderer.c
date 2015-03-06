#include "kinect_renderer.h"
#include <zlib.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>
#include <bot_frames/bot_frames.h>

#include <lcmtypes/kinect_frame_msg_t.h>
#include <kinect/kinect-utils.h>

#include "jpeg-utils-ijg.h"

#define PARAM_HISTORY_FREQUENCY "Scan Hist. Freq."
#define PARAM_ALPHA "Alpha."
#define MAX_REFERSH_RATE_USEC 30000 //not sure whether we really need this part

typedef struct _KinectRenderer {
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer   *viewer;
    lcm_t     *lcm;
    BotFrames *frames;
    char * kinect_frame;

    kinect_frame_msg_t* msg;

    int64_t last_kinect_data_utime;

    int width;
    int height;

    // raw disparity
    uint16_t* disparity;
    int need_to_recompute_frame_data;

    uint8_t* uncompress_buffer;
    int uncompress_buffer_size;

    uint8_t* rgb_data;

    KinectCalibration* kcal;
} KinectRenderer;

static void
recompute_frame_data(KinectRenderer* self)
{
    if(!self->msg) {
        self->need_to_recompute_frame_data = 0;
        return;
    }

    int npixels = self->width * self->height;

    const uint8_t* depth_data = self->msg->depth.depth_data;

    if(self->msg->depth.compression != KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
        if(self->msg->depth.uncompressed_size > self->uncompress_buffer_size) {
            self->uncompress_buffer_size = self->msg->depth.uncompressed_size;
            self->uncompress_buffer = (uint8_t*) realloc(self->uncompress_buffer, self->uncompress_buffer_size);
        }
        unsigned long dlen = self->msg->depth.uncompressed_size;
        int status = uncompress(self->uncompress_buffer, &dlen, 
                self->msg->depth.depth_data, self->msg->depth.depth_data_nbytes);
        if(status != Z_OK) {
            return;
        }
        depth_data = self->uncompress_buffer;
    }

    switch(self->msg->depth.depth_data_format) {
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT:
            if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
                int16_t* rdd = (int16_t*) depth_data;
                int i;
                for(i=0; i<npixels; i++) {
                    int d = rdd[i];
                    self->disparity[i] = d;
                }
            } else {
                fprintf(stderr, "Big endian systems not supported\n");
            }
            break;
    case KINECT_DEPTH_MSG_T_DEPTH_MM:
        if(G_BYTE_ORDER == G_LITTLE_ENDIAN) {
            int16_t* rdd = (int16_t*) depth_data;
            int i;
            for(i=0; i<npixels; i++) {
                int d = rdd[i];
                self->disparity[i] = d;
            }
        } else {
            fprintf(stderr, "Big endian systems not supported\n");
        }
        break;
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT:
        fprintf(stderr, "10-bit depth data not supported\n");
        break;
        default:
            break;
    }
}

static void 
on_kinect_frame (const lcm_recv_buf_t *rbuf, const char *channel,
        const kinect_frame_msg_t *msg, void *user_data )
{
    KinectRenderer *self = (KinectRenderer*) user_data;

    static int64_t last_redraw_utime = 0;
    int64_t now = bot_timestamp_now(); 

    double hist_spc = bot_gtk_param_widget_get_double (self->pw, PARAM_HISTORY_FREQUENCY);

    if (abs (msg->timestamp - self->last_kinect_data_utime) > (int64_t)(1E6/hist_spc)) {
        self->last_kinect_data_utime = msg->timestamp;

        if(self->msg)
            kinect_frame_msg_t_destroy(self->msg);
        self->msg = kinect_frame_msg_t_copy(msg);

        // TODO check width, height

        if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
            memcpy(self->rgb_data, msg->image.image_data, 
                   self->width * self->height * 3);
        } else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
            jpegijg_decompress_8u_rgb(msg->image.image_data, msg->image.image_data_nbytes,
                                      self->rgb_data, self->width, self->height, self->width * 3);
        }

        self->need_to_recompute_frame_data = 1;

        if ((now - last_redraw_utime) > MAX_REFERSH_RATE_USEC) {
            bot_viewer_request_redraw( self->viewer );
            last_redraw_utime = now;
        }

        bot_viewer_request_redraw(self->viewer);
    }
    //else - skip 
}

static void on_param_widget_changed (BotGtkParamWidget *pw, const char *name, void *user)
{
    KinectRenderer *self = (KinectRenderer*) user;

    // TODO

    bot_viewer_request_redraw(self->viewer);
}

static void
on_load_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    KinectRenderer *self = user_data;
    bot_gtk_param_widget_load_from_key_file (self->pw, keyfile, self->kinect_frame);
}

static void
on_save_preferences (BotViewer *viewer, GKeyFile *keyfile, void *user_data)
{
    KinectRenderer *self = user_data;
    bot_gtk_param_widget_save_to_key_file (self->pw, keyfile, self->kinect_frame);
}

static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
        double result[3])
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}

static inline void
_matrix_transpose_4x4d (const double m[16], double result[16])
{
    result[0] = m[0];
    result[1] = m[4];
    result[2] = m[8];
    result[3] = m[12];
    result[4] = m[1];
    result[5] = m[5];
    result[6] = m[9];
    result[7] = m[13];
    result[8] = m[2];
    result[9] = m[6];
    result[10] = m[10];
    result[11] = m[14];
    result[12] = m[3];
    result[13] = m[7];
    result[14] = m[11];
    result[15] = m[15];
}

static void _draw(BotViewer *viewer, BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer->user;
    if(!self->msg)
        return;

    if(self->need_to_recompute_frame_data)
        recompute_frame_data(self);

    glPushMatrix();
    if (self->frames==NULL || !bot_frames_have_trans(self->frames,self->kinect_frame,bot_frames_get_root_name(self->frames))){
      // rotate so that X is forward and Z is up
      glRotatef(-90, 0, 0, 1);
      glRotatef(-90, 1, 0, 0);
    }
    else{
      //project to current frame
      double kinect_to_local_m[16];
      bot_frames_get_trans_mat_4x4_with_utime(self->frames,self->kinect_frame,bot_frames_get_root_name(self->frames),
          self->msg->timestamp, kinect_to_local_m);
      // opengl expects column-major matrices
      double kinect_to_local_m_opengl[16];
      bot_matrix_transpose_4x4d(kinect_to_local_m, kinect_to_local_m_opengl);
      glMultMatrixd(kinect_to_local_m_opengl);
    }

    //fprintf(stderr,"Depth Type : %d => %d \n", self->msg->depth.depth_data_format, KINECT_DEPTH_MSG_T_DEPTH_11BIT);//KINECT_DEPTH_MSG_T_DEPTH_MM);
    //float so = self->kcal->shift_offset; //unused
    double alpha = bot_gtk_param_widget_get_double (self->pw, PARAM_ALPHA);
    
    double depth_to_rgb_uvd[12];
    double depth_to_depth_xyz[16];

    kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(self->kcal, depth_to_rgb_uvd);
    kinect_calib_get_depth_uvd_to_depth_xyz_4x4(self->kcal, depth_to_depth_xyz);

    double depth_to_depth_xyz_trans[16];
    _matrix_transpose_4x4d(depth_to_depth_xyz, depth_to_depth_xyz_trans);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if(self->msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_11BIT ){
        glPushMatrix();
        glMultMatrixd(depth_to_depth_xyz_trans);

        glEnable(GL_DEPTH_TEST);
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        glColor3f(0, 0, 0);
        for(int u=0; u<self->width; u++) {
            for(int v=0; v<self->height; v++) {
                uint16_t disparity = self->disparity[v*self->width+u];

                double uvd_depth[4] = { u, v, disparity, 1 };
                double uvd_rgb[3];
                _matrix_vector_multiply_3x4_4d(depth_to_rgb_uvd, uvd_depth, uvd_rgb);

                double uv_rect[2] = {
                    uvd_rgb[0] / uvd_rgb[2],
                    uvd_rgb[1] / uvd_rgb[2]
                };
                double uv_dist[2];

                // compute distorted pixel coordinates
                kinect_calib_distort_rgb_uv(self->kcal, uv_rect, uv_dist);
                int u_rgb = uv_dist[0] + 0.5;
                int v_rgb = uv_dist[1] + 0.5;

                uint8_t r, g, b;
                if(u_rgb >= self->width || u_rgb < 0 || v_rgb >= self->height || v_rgb < 0) {
                    r = g = b = 0;
                } else {
                    r = self->rgb_data[v_rgb*self->width*3 + u_rgb*3 + 0];
                    g = self->rgb_data[v_rgb*self->width*3 + u_rgb*3 + 1];
                    b = self->rgb_data[v_rgb*self->width*3 + u_rgb*3 + 2];
                }

                glColor4f(r / 255.0, g / 255.0, b / 255.0, alpha);

                glVertex3f(u, v, disparity);
            }
        }
        glEnd();

        glPopMatrix();
    }
    else if(self->msg->depth.depth_data_format == KINECT_DEPTH_MSG_T_DEPTH_MM){
        double fx_inv = 1/ self->kcal->intrinsics_depth.fx;
        double cx = self->kcal->intrinsics_depth.cx;
        double cy = self->kcal->intrinsics_depth.cy;
        
        glEnable(GL_DEPTH_TEST);
        glPointSize(2.0f);
        glBegin(GL_POINTS);
        glColor3f(0, 0, 0);
        for(int u=0; u<self->width; u++) {
            for(int v=0; v<self->height; v++) {
                uint16_t disparity = self->disparity[v*self->width+u];
                float depth = disparity / 1000.0;

                uint8_t r, g, b;
                r = self->rgb_data[v*self->width*3 + u*3 + 0];
                g = self->rgb_data[v*self->width*3 + u*3 + 1];
                b = self->rgb_data[v*self->width*3 + u*3 + 2];

                //glColor3f(r / 255.0, g / 255.0, b / 255.0);
                glColor4f(r / 255.0, g / 255.0, b / 255.0, alpha);
                float x = ( u * fx_inv - cx* fx_inv) * depth;
                float y = ( v * fx_inv - cy* fx_inv) * depth;

                glVertex3f(x,y, depth);
            }
	  
	  //float disparity =  1000 * 0.07214 * 576.09757860 / self->disparity[v*self->width+u] / 8;
	  //glColor3f(255.0 / 255.0, 0 / 255.0, 0 / 255.0);
	  //glVertex3f(u, v, disparity);
        }
        glEnd();
    }
    /*
    glPointSize(2.1f);
    glBegin(GL_POINTS);
    for ( int u = 0; u < 640; u++ ) {
      for ( int v = 0; v < 480; v++ ) {
	if (self->disparity[v*640 + u] == 2047) continue;

	double pix[] = { u, v, self->disparity[v*640 + u], 1.0 };
	double xyz[4];
	bot_matrix_vector_multiply_4x4_4d (depth_to_depth_xyz, pix, xyz);

	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(xyz[0]/xyz[3], xyz[1]/xyz[3], xyz[2]/xyz[3]);
      }
    }
    glEnd();
    */
    glPopMatrix(); //kinect_to_local
    glDisable(GL_BLEND);

}

static void _free(BotRenderer *renderer)
{
    KinectRenderer *self = (KinectRenderer*) renderer;

    if(self->msg)
        kinect_frame_msg_t_destroy(self->msg);

    free(self->uncompress_buffer);
    self->uncompress_buffer_size = 0;

    kinect_calib_destroy(self->kcal);

    free(self->disparity);
    free(self->rgb_data);
    if(self->kinect_frame)
      free(self->kinect_frame);

    free(self);
}

inline void fillDefault(KinectRenderer *self, double* R, double* T)
{
    self->kcal->width = 640;
    self->kcal->height = 480;
    self->kcal->intrinsics_depth.fx = 576.09757860;
    self->kcal->intrinsics_depth.cx = 321.06398107;
    self->kcal->intrinsics_depth.cy = 242.97676897;

    self->kcal->intrinsics_rgb.fx = 528.49404721;
    self->kcal->intrinsics_rgb.cx = 319.50000000;
    self->kcal->intrinsics_rgb.cy = 239.50000000;
    self->kcal->intrinsics_rgb.k1 = 0;
    self->kcal->intrinsics_rgb.k2 = 0;

    self->kcal->shift_offset = 1093.4753;
    self->kcal->projector_depth_baseline = 0.07214;

    R[0] = 0.999999;
    R[1] = -0.000796;
    R[2] = 0.001256;
    R[3] = 0.000739;
    R[4] = 0.998970;
    R[5] = 0.045368;
    R[6] = -0.001291;
    R[7] = -0.045367;
    R[8] = 0.998970;

    T[0] = -0.015756;
    T[1] = -0.000923;
    T[2] = 0.002316;
}

void 
kinect_add_renderer_to_viewer(BotViewer* viewer, int priority, lcm_t* lcm, BotFrames * frames, 
                              const char * kinect_frame, BotParam *param)
{
    KinectRenderer *self = (KinectRenderer*) calloc(1, sizeof(KinectRenderer));
    char* channel_name;
    char default_channel_name[] = "KINECT_FRAME";
    char prefix[256], temp[512];

    self->need_to_recompute_frame_data = 0;
    self->width = 640;
    self->height = 480;
    self->disparity = (uint16_t*) malloc(self->width * self->height * sizeof(uint16_t));
    self->rgb_data = (uint8_t*) malloc(self->width * self->height * 3);

    self->frames = frames;
    if ( kinect_frame )
      self->kinect_frame = strdup(kinect_frame);

    self->msg = NULL;

    BotRenderer *renderer = &self->renderer;

    double R[9];
    double T[3];
    self->kcal = kinect_calib_new();

    if (param) {
        char **rgbd_names = bot_param_get_subkeys (param, "rgbd_cameras");
        if (rgbd_names) {
            if (rgbd_names[0]) {
                int checkInt = 0;
                int useDefault = 0;
		sprintf(prefix, "rgbd_cameras.%s.", self->kinect_frame);

		strcpy(temp, prefix);
                self->kcal->width = bot_param_get_int_or_fail(param, strcat(temp, "intrinsic_cal.width") );
		strcpy(temp, prefix);
                self->kcal->height = bot_param_get_int_or_fail(param, strcat(temp, "intrinsic_cal.height") );

                if (self->kcal->width == 0 || self->kcal->height == 0)
                    useDefault = 1;

		strcpy(temp, prefix);
                self->kcal->intrinsics_depth.fx = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.depth_fx"));
		strcpy(temp, prefix);
                self->kcal->intrinsics_depth.cx = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.depth_cx"));
		strcpy(temp, prefix);
                self->kcal->intrinsics_depth.cy = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.depth_cy"));

                if (self->kcal->intrinsics_depth.cx == 0
                    || self->kcal->intrinsics_depth.cy == 0
                    || self->kcal->intrinsics_depth.fx == 0)
                    useDefault = 1;

		strcpy(temp, prefix);
                self->kcal->intrinsics_rgb.fx = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.rgb_fx"));
		strcpy(temp, prefix);
                self->kcal->intrinsics_rgb.cx = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.rgb_cx"));
		strcpy(temp, prefix);
                self->kcal->intrinsics_rgb.cy = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.rgb_cy"));
		strcpy(temp, prefix);
                self->kcal->intrinsics_rgb.k1 = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.rgb_k1"));
		strcpy(temp, prefix);
                self->kcal->intrinsics_rgb.k2 = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.rgb_k2"));

                 if (self->kcal->intrinsics_rgb.cx == 0 
                    || self->kcal->intrinsics_rgb.cy == 0
                    || self->kcal->intrinsics_rgb.fx == 0   
                    || self->kcal->intrinsics_rgb.k1 == 0
                    || self->kcal->intrinsics_rgb.k2 == 0)
                    useDefault = 1;

		 strcpy(temp, prefix);
		 self->kcal->shift_offset = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.shift_offset"));
		 strcpy(temp, prefix);
		 self->kcal->projector_depth_baseline = bot_param_get_double_or_fail(param, strcat(temp, "intrinsic_cal.projector_depth_baseline"));

                if (self->kcal->shift_offset == 0 
                    || self->kcal->projector_depth_baseline == 0)
                    useDefault = 1;

		strcpy(temp, prefix);
                bot_param_get_double_array_or_fail(param, strcat(temp, "intrinsic_cal.R"), R, 9);
		strcpy(temp, prefix);
		bot_param_get_double_array_or_fail(param, strcat(temp, "intrinsic_cal.T"), T, 3);

                if (useDefault) {
                    fprintf(stderr, "Error reading specific KINECT info - using default values\n");
                    fillDefault(self, R, T);
                }
            }
            else {
                fprintf(stderr, "Error reading kinect info (rgbd_cameras key not available) - using default values\n");
                fillDefault(self, R, T);
            }
        }
        else {
            fprintf(stderr, "Error reading from param server - using default values\n");
            fillDefault(self, R, T);
        }
        g_strfreev (rgbd_names);
    }
    else {
        fprintf(stderr, "Error: Param server is null - using default values\n");
        fillDefault(self, R, T);
    }
    
            

    memcpy(self->kcal->depth_to_rgb_rot, R, 9*sizeof(double));
    memcpy(self->kcal->depth_to_rgb_translation, T, 3*sizeof(double));

    self->lcm = lcm;
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());

    bot_gtk_param_widget_add_double(self->pw, PARAM_HISTORY_FREQUENCY, 
                                    BOT_GTK_PARAM_WIDGET_SLIDER, 
                                    0.1, 30, 0.1, 30.0);

    bot_gtk_param_widget_add_double(self->pw, PARAM_ALPHA, 
                                    BOT_GTK_PARAM_WIDGET_SLIDER, 
                                    0.05, 1.f, 0.05, 1.0);

    self->uncompress_buffer = NULL;
    self->uncompress_buffer_size = 0;

    renderer->draw = _draw;
    renderer->destroy = _free;
    renderer->name = self->kinect_frame;
    renderer->widget = GTK_WIDGET(self->pw);
    renderer->enabled = 1;
    renderer->user = self;

    g_signal_connect (G_OBJECT (self->pw), "changed",
                      G_CALLBACK (on_param_widget_changed), self);
    g_signal_connect (G_OBJECT (viewer), "load-preferences",
                      G_CALLBACK (on_load_preferences), self);
    g_signal_connect (G_OBJECT (viewer), "save-preferences",
                      G_CALLBACK (on_save_preferences), self);

    sprintf(prefix, "rgbd_cameras.%s.lcm_channel", self->kinect_frame);

    channel_name = default_channel_name;
    if ( param ) {
      if ( bot_param_get_str(param, prefix, &channel_name) == -1 ) {
	kinect_frame_msg_t_subscribe(self->lcm, "KINECT_FRAME", on_kinect_frame, self);
      } else {
	kinect_frame_msg_t_subscribe(self->lcm, channel_name, on_kinect_frame, self);
	free(channel_name);
      }
    } else {
      kinect_frame_msg_t_subscribe(self->lcm, "KINECT_FRAME", on_kinect_frame, self);
    }

    bot_viewer_add_renderer(viewer, renderer, priority);
}
