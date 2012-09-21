#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <glib.h>

#include <camunits/cam.h>
#include <camunits/plugin.h>

typedef struct _StereoSplitStereo StereoSplitStereo;
typedef struct _StereoSplitStereoClass StereoSplitStereoClass;

// run-time type safe replacement for casting with (StereoSplitStereo*)
#define STEREO_SPLIT_STEREO(obj)  (G_TYPE_CHECK_INSTANCE_CAST( (obj), \
        stereo_split_stereo_get_type(), StereoSplitStereo))

void cam_plugin_initialize (GTypeModule * module);
CamUnitDriver * cam_plugin_create (GTypeModule * module);
struct _StereoSplitStereo {
    CamUnit parent;
    CamFrameBuffer * outbuf;
};

struct _StereoSplitStereoClass {
    CamUnitClass parent_class;
};

GType stereo_split_stereo_get_type (void);

static StereoSplitStereo * stereo_split_stereo_new(void);
static void on_input_frame_ready (CamUnit * super, const CamFrameBuffer *inbuf,
        const CamUnitFormat *infmt);
static void on_input_format_changed (CamUnit *super, 
        const CamUnitFormat *infmt);

CAM_PLUGIN_TYPE (StereoSplitStereo, stereo_split_stereo, CAM_TYPE_UNIT);

/* These next two functions are required as entry points for the
 * plug-in API. */
void
cam_plugin_initialize (GTypeModule * module)
{
    stereo_split_stereo_register_type (module);
}

CamUnitDriver *
cam_plugin_create (GTypeModule * module)
{
    return cam_unit_driver_new_stock_full ("stereo", 
            "split_bumblebee2",
            "Split Bumblebee2 Stereo Image", 0, 
            (CamUnitConstructor)stereo_split_stereo_new,
            module);
}

// ============== StereoSplitStereo ===============
static void on_input_frame_ready (CamUnit * super, const CamFrameBuffer *inbuf,
        const CamUnitFormat *infmt);
static void on_input_format_changed (CamUnit *super, 
        const CamUnitFormat *infmt);
static void _finalize (GObject * obj);

static void
stereo_split_stereo_class_init (StereoSplitStereoClass *klass)
{
    klass->parent_class.on_input_frame_ready = on_input_frame_ready;
    GObjectClass * gobject_class = G_OBJECT_CLASS (klass);
    gobject_class->finalize = _finalize;
}

static void
_finalize (GObject * obj)
{
    StereoSplitStereo * self = STEREO_SPLIT_STEREO(obj);
    if(self->outbuf)
        g_object_unref(self->outbuf);
    G_OBJECT_CLASS (stereo_split_stereo_parent_class)->finalize(obj);
}

static void
stereo_split_stereo_init (StereoSplitStereo *self)
{
    self->outbuf = NULL;
    g_signal_connect (G_OBJECT(self), "input-format-changed",
            G_CALLBACK(on_input_format_changed), NULL);
}

StereoSplitStereo * 
stereo_split_stereo_new()
{
    return STEREO_SPLIT_STEREO(g_object_new(stereo_split_stereo_get_type(), NULL));
}

static void 
on_input_frame_ready (CamUnit *super, const CamFrameBuffer *inbuf, 
        const CamUnitFormat *infmt)
{
    StereoSplitStereo *self = STEREO_SPLIT_STEREO(super);

    const CamUnitFormat *outfmt = cam_unit_get_output_format(super);
    int out_stride = outfmt->row_stride;
    uint8_t * left_img_start = self->outbuf->data;
    uint8_t * right_img_start = self->outbuf->data + 
        infmt->height * out_stride;

    for (int i=0; i<infmt->height; i++) {
        uint8_t * left_row  = left_img_start  + i * out_stride;
        uint8_t * right_row = right_img_start + i * out_stride;
        uint8_t * src_row = inbuf->data + i * infmt->row_stride;

        for (int j=0; j<infmt->width; j++) {
            left_row[j]  = src_row[j * 2];
            right_row[j] = src_row[j * 2 + 1];
        }
    }
    self->outbuf->timestamp = inbuf->timestamp;
    self->outbuf->bytesused = self->outbuf->length;
    cam_unit_produce_frame (super, self->outbuf, outfmt);
}

static void
on_input_format_changed (CamUnit *super, const CamUnitFormat *infmt)
{
    StereoSplitStereo *self = STEREO_SPLIT_STEREO(super);
    cam_unit_remove_all_output_formats (super);

    if (!infmt) return;

    if (infmt->pixelformat != CAM_PIXEL_FORMAT_BE_GRAY16)
        return;

    int width = infmt->width;
    int height = infmt->height * 2;
    int row_stride = width;
    int buf_sz = height * row_stride;

    if(self->outbuf) {
        g_object_unref(self->outbuf);
    }
    self->outbuf = cam_framebuffer_new_alloc(buf_sz);

    cam_unit_add_output_format (super, CAM_PIXEL_FORMAT_GRAY,
            NULL, infmt->width, height, 
            row_stride);
}
