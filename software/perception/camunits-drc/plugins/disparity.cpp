#include <camunits/plugin.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>

typedef struct _CamcvDisparity CamcvDisparity;
struct _CamcvDisparity {
    CamUnit parent;

    CamUnitControl * stereo_preset;
    CamUnitControl * min_disparity;
    CamUnitControl * num_disparities;
    CamUnitControl * sad_win_size;

    CamFrameBuffer * outbuf;
    cv::Ptr<cv::Mat> left_frame;
    cv::Ptr<cv::Mat> right_frame;
    cv::Ptr<cv::Mat> dst_cv;
    cv::Ptr<cv::StereoBM> stereo_;
};

typedef struct _CamcvDisparityClass CamcvDisparityClass;
struct _CamcvDisparityClass {
    CamUnitClass parent_class;
};

static CamcvDisparity * camcv_disparity_new(void);
static void on_input_frame_ready(CamUnit * super, const CamFrameBuffer *inbuf,
        const CamUnitFormat *infmt);
static void on_input_format_changed(CamUnit *super,
        const CamUnitFormat *infmt);
static int _stream_init(CamUnit * super, const CamUnitFormat * format);
static int _stream_shutdown(CamUnit * super);
static gboolean _try_set_control(CamUnit *super, const CamUnitControl *ctl,
        const GValue *proposed, GValue *actual);
static void _finalize (GObject * obj);

int SAD_setter(const GValue *proposed);
int num_disparities_setter(const GValue *proposed);

GType camcv_disparity_get_type(void);
CAM_PLUGIN_TYPE(CamcvDisparity, camcv_disparity, CAM_TYPE_UNIT);


extern "C"{
	/* These next two functions are required as entry points for the
	 * plug-in API. */
	void cam_plugin_initialize(GTypeModule * module);
	void cam_plugin_initialize(GTypeModule * module)
	{
		camcv_disparity_register_type(module);
	}

	CamUnitDriver * cam_plugin_create(GTypeModule * module);
	CamUnitDriver * cam_plugin_create(GTypeModule * module)
	{
		return cam_unit_driver_new_stock_full("bumblebee2", "disparity",
				"Compute Disparity Map", 0,
				(CamUnitConstructor)camcv_disparity_new, module);
	}
}

static void
camcv_disparity_class_init(CamcvDisparityClass *klass)
{
    klass->parent_class.on_input_frame_ready = on_input_frame_ready;
    klass->parent_class.stream_init = _stream_init;
    klass->parent_class.stream_shutdown = _stream_shutdown;
    klass->parent_class.try_set_control = _try_set_control;

    GObjectClass * gobject_class = G_OBJECT_CLASS (klass);
    gobject_class->finalize = _finalize;
}

static void
_finalize (GObject * obj)
{
    CamcvDisparity * self = (CamcvDisparity*)(obj);
    if(self->outbuf)
       g_object_unref(self->outbuf);
    G_OBJECT_CLASS (camcv_disparity_parent_class)->finalize(obj);
}

static void
camcv_disparity_init(CamcvDisparity *self)
{
	CamUnit *unit = CAM_UNIT(self);
	self->sad_win_size = cam_unit_add_control_int(unit, "sad_win_size", "SAD Window Size", 5, 255, 2, 21, 1);
	self->num_disparities = cam_unit_add_control_int(unit, "num_disparities", "Number of Disparities", 0, 256, 16, 0, 1);
	self->min_disparity = cam_unit_add_control_int(unit, "min_disparity", "Minimum Disparity", 0, 256, 1, 0, 1);

    CamUnitControlEnumValue stereo_presets[] = {
        { cv::StereoBM::BASIC_PRESET, "Basic", 1 },
        { cv::StereoBM::NARROW_PRESET, "Narrow", 1 },
        { cv::StereoBM::FISH_EYE_PRESET, "Fisheye", 1 },
        { -1, NULL, 0 }
    };
	self->stereo_preset = cam_unit_add_control_enum(unit, "stereo_preset", "Stereo Preset", cv::StereoBM::BASIC_PRESET, 1, stereo_presets);

    self->outbuf = NULL;
    self->left_frame = NULL;
    self->right_frame = NULL;
    self->dst_cv = NULL;

    self->stereo_ = new cv::StereoBM();

    g_signal_connect(G_OBJECT(self), "input-format-changed",
            G_CALLBACK(on_input_format_changed), self);
}

static CamcvDisparity *
camcv_disparity_new()
{
    return (CamcvDisparity*)(g_object_new(camcv_disparity_get_type(), NULL));
}

static int
_stream_init(CamUnit * super, const CamUnitFormat * fmt)
{
    //CamcvDisparity *self = (CamcvDisparity*)(super);

     return 0;
}

static int
_stream_shutdown(CamUnit * super)
{
    //CamcvDisparity *self = (CamcvDisparity*)(super);

    return 0;
}

static void
on_input_frame_ready(CamUnit *super, const CamFrameBuffer *inbuf,
        const CamUnitFormat *infmt)
{
    CamcvDisparity *self = (CamcvDisparity*)(super);

    if (!infmt) return;
    if (infmt->pixelformat != CAM_PIXEL_FORMAT_GRAY)return;

    for(int r=0; r<infmt->height/2; r++) {
        memcpy(self->left_frame->ptr(r),
                inbuf->data + r * infmt->row_stride,
                infmt->width);
        memcpy(self->right_frame->ptr(r),
                inbuf->data + (r + infmt->height/2) * infmt->row_stride,
                infmt->width);
    }

    self->stereo_->operator ()(*self->left_frame, *self->right_frame, *self->dst_cv);

    self->outbuf->timestamp = inbuf->timestamp;
    const CamUnitFormat *outfmt = cam_unit_get_output_format(super);
    self->outbuf->bytesused = outfmt->height * outfmt->row_stride;

//    Dumps disparity image to raw file
//    std::ofstream ofile;
//    ofile.open("disparity16.raw",std::ios_base::binary);
//
//    ofile.write((char *)&outfmt->width, sizeof(int));
//    ofile.write((char *)&outfmt->height, sizeof(int));
//
//    ofile.write((char *)self->outbuf->data, self->outbuf->bytesused);
//
//	ofile.close();

    cam_unit_produce_frame(super, self->outbuf, outfmt);
}

static void
on_input_format_changed(CamUnit *super, const CamUnitFormat *infmt)
{
    CamcvDisparity *self = (CamcvDisparity*)(super);
    cam_unit_remove_all_output_formats(CAM_UNIT(self));

    if(!infmt) return;
    if (infmt->pixelformat != CAM_PIXEL_FORMAT_GRAY)return;

    if(self->outbuf)
        g_object_unref(self->outbuf);
    cv::Size img_size(infmt->width, infmt->height/2);

    self->left_frame = new cv::Mat(img_size, CV_8U);
    self->right_frame = new cv::Mat(img_size, CV_8U);
    self->dst_cv = new cv::Mat(img_size, CV_16S);

    // dst_data is of type int8_t * here even though it will point to 16 bit
    // data. This is just used here so that it can be passed to cam_framebuffer_new
    uint8_t * dst_data = NULL;
    dst_data = (uint8_t *)self->dst_cv->data;
    int out_stride = self->dst_cv->step;
    int buf_sz = img_size.height * out_stride;
    self->outbuf = cam_framebuffer_new(dst_data, buf_sz);
    self->outbuf->bytesused = buf_sz;

    cam_unit_add_output_format(CAM_UNIT(self),
    		CAM_PIXEL_FORMAT_BE_SIGNED_GRAY16,
            NULL, img_size.width, img_size.height, out_stride);
}

static gboolean
_try_set_control(CamUnit *super, const CamUnitControl *ctl,
        const GValue *proposed, GValue *actual) {
	CamcvDisparity *self = (CamcvDisparity*)(super);

	int sad_win_size = cam_unit_control_get_int(self->sad_win_size);
    int num_disparities = cam_unit_control_get_int(self->num_disparities);

	if (ctl == self->sad_win_size)	{
		sad_win_size = SAD_setter(proposed);
		g_value_set_int(actual, sad_win_size);
	}
	else if (ctl == self->num_disparities){
		num_disparities = num_disparities_setter(proposed);
		g_value_set_int(actual, num_disparities);
	}
	else{
		g_value_copy(proposed, actual);
	}


    int stereo_preset = cam_unit_control_get_enum(self->stereo_preset);
//    int num_disparities = (ctl==self->num_disparities)?g_value_get_int(proposed):cam_unit_control_get_int(self->num_disparities);
//    int sad_win_size = (ctl==self->sad_win_size)?g_value_get_int(proposed):cam_unit_control_get_int(self->sad_win_size);
//    int stereo_preset = (ctl==self->stereo_preset)?g_value_get_int(proposed):cam_unit_control_get_enum(self->stereo_preset);

    self->stereo_ = new cv::StereoBM(stereo_preset, num_disparities, sad_win_size);

    self->stereo_->state->minDisparity = cam_unit_control_get_int(self->min_disparity);

    return TRUE;
}

int SAD_setter(const GValue *proposed){
	//Constraints are: SADWindowSize must be odd, be within 5..255 and be not larger than image width or height
	int proposed_value = g_value_get_int(proposed);

	int 	value = (proposed_value%2)==1?proposed_value:proposed_value+1; //must be odd;

	value = std::min(std::max(5, value),255); //bounds of 5..255;

	return value;
}

int num_disparities_setter(const GValue *proposed){
	//numberOfDisparities must be positive and divisble by 16
	int proposed_value = g_value_get_int(proposed);

	int 	value = (proposed_value%16)==0? proposed_value: proposed_value - (proposed_value%16); //must be divisible by 16;

	value = std::max(0, value); //bounds of 5..255;

	return value;
}
