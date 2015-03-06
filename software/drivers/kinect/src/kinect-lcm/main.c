#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>

#include <pthread.h>

#include <zlib.h>
#include <glib.h>
#include <lcm/lcm.h>
#include <libfreenect.h>

#include <lcmtypes/kinect_depth_msg_t.h>
#include <lcmtypes/kinect_image_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/kinect_cmd_msg_t.h>
#include <lcmtypes/kinect_sensor_status_t.h>

#if USE_JPEG_UTILS_POD
#include <jpeg-utils/jpeg.h>
#else
#include "jpeg-utils-ijg.h"
#endif

#include "timestamp.h"
#include "pixels.h"

#define dbg(...) fprintf(stderr, __VA_ARGS__)

#define alpha 0.05 //decay rate for the moving average
typedef struct _rate_t {
    double target_hz;
    double current_hz;
    int64_t last_tick;
    int64_t tick_count;
} rate_t;

rate_t* rate_new(double target_hz);

void rate_destroy(rate_t* rate);

/**
 * returns: 1 if an image should be published.  0 if not
 */
int rate_check(rate_t* rate);

typedef struct _state_t {
    GThread* freenect_thread;
    volatile int die;

    freenect_context *f_ctx;
    freenect_device *f_dev;
    int freenect_angle;
    int freenect_led;

    int8_t requested_image_format;
    int8_t current_image_format;

    int8_t requested_depth_format;
    int8_t current_depth_format;

    int8_t current_resolution;

    uint8_t* image_data;

    uint8_t* image_buf;
    int image_buf_size;

    uint8_t* debayer_buf;
    int debayer_buf_size;

    uint16_t* depth_unpack_buf;
    int depth_unpack_buf_size;

    uint8_t* depth_compress_buf;
    int debayer_buf_stride;
    int depth_compress_buf_size;

    freenect_raw_tilt_state* tilt_state;
    double accel_mks[3];
    double tilt_radians;

    int8_t current_led;
    int8_t requested_led;

    kinect_frame_msg_t msg;
    char* msg_channel;

    int got_img;
    int got_depth;

    int skip_img;
    int skip_depth;
    int throttle; //1 in data skip will be published
    timestamp_sync_state_t* clocksync;

    rate_t* capture_rate;
    rate_t* report_rate;

    uint8_t* jpeg_buf;
    int jpeg_buf_size;

    int jpeg_quality;

    int use_zlib;

    lcm_t* lcm;

    int64_t last_timestamp;
    int64_t last_depth_timestamp;
    int64_t last_img_timestamp;

    pthread_t  work_thread;
    
} state_t;

static void
populate_status(state_t* state, kinect_frame_msg_t* msg, int64_t timestamp)
{
    freenect_update_tilt_state(state->f_dev);
    state->tilt_state = freenect_get_tilt_state(state->f_dev);

    double dx, dy, dz;
    freenect_get_mks_accel(state->tilt_state, &dx, &dy, &dz);

    msg->timestamp = timestamp;
    msg->raw_accel[0] = state->tilt_state->accelerometer_x;
    msg->raw_accel[1] = state->tilt_state->accelerometer_y;
    msg->raw_accel[2] = state->tilt_state->accelerometer_z;
    msg->raw_tilt = state->tilt_state->tilt_angle;

    msg->accel[0] = dx;
    msg->accel[1] = dy;

    // XXX the kinect accelerometer appears to use a left-handed coordinate
    // frame, where Z points in towards the camera.  We could invert it here 
    // to make it more consistent with standard coordinate frames.
    msg->accel[2] = dz;

    msg->tilt_radians = freenect_get_tilt_degs(state->tilt_state) * M_PI / 180;

    msg->led_status = state->current_led;

    switch (state->tilt_state->tilt_status) {
    case TILT_STATUS_STOPPED:
        msg->tilt_status = KINECT_FRAME_MSG_T_TILT_STATUS_STOPPED;
        break;
    case TILT_STATUS_LIMIT:
        msg->tilt_status = KINECT_FRAME_MSG_T_TILT_STATUS_LIMIT;
        break;
    case TILT_STATUS_MOVING:
        msg->tilt_status = KINECT_FRAME_MSG_T_TILT_STATUS_MOVING;
        break;
    }
    msg->tilt_status = state->tilt_state->tilt_status;
}

rate_t* rate_new(double target_hz)
{
    rate_t* rt = (rate_t *) calloc(1, sizeof(rate_t));
    rt->target_hz = target_hz;
    rt->tick_count = 0;
    return rt;
}

void rate_destroy(rate_t* rate)
{
    free(rate);
}

int rate_check(rate_t* rate)
{
    // check the current time
    int64_t c_utime = timestamp_now();

    // compute the framerate if we were to publish an image
    int64_t dt = c_utime - rate->last_tick;

    double p_framerate = alpha * (1.0 * 1e6 / dt) + (1 - alpha) * rate->current_hz;
    if (p_framerate > rate->target_hz) {
        // if the potential framerate is too high, don't publish, and return 0
        return 0;
    }
    else {
        // otherwise, update current_hz with a exponential moving average, and return 1
        rate->current_hz = p_framerate;
        rate->last_tick = c_utime;
        rate->tick_count++;
        return 1;
    }
}

static void
set_image_depth_formats(state_t* state)
{
    freenect_video_format vfmt;
    freenect_depth_format dfmt;

    switch (state->requested_image_format) {
    case KINECT_IMAGE_MSG_T_VIDEO_RGB:
    case KINECT_IMAGE_MSG_T_VIDEO_BAYER:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_8BIT:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT_PACKED:
    case KINECT_IMAGE_MSG_T_VIDEO_YUV_RAW:
        {
            vfmt = state->requested_image_format;
            freenect_frame_mode md = freenect_find_video_mode(state->current_resolution, state->requested_image_format);
            state->msg.image.image_data_nbytes = md.bytes;
        }
        break;
    case KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG:
        vfmt = FREENECT_VIDEO_BAYER;
        break;
    default:
        vfmt = FREENECT_VIDEO_BAYER;
        fprintf(stderr, "Invalid image format requested: %d\n", state->requested_image_format);
        state->msg.image.image_data_nbytes = 0;
        break;
    }

    switch (state->requested_depth_format) {
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT:
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT:
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT_PACKED:
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT_PACKED:
    case KINECT_DEPTH_MSG_T_DEPTH_REGISTERED:
    case KINECT_DEPTH_MSG_T_DEPTH_MM:
        dfmt = state->requested_depth_format;
        break;
    default:
        dfmt = FREENECT_DEPTH_11BIT_PACKED;
        fprintf(stderr, "Invalid depth format requested: %d\n", state->requested_depth_format);
        break;
    }

    state->current_image_format = state->requested_image_format;
    state->msg.image.image_data_format = state->requested_image_format;
    state->current_depth_format = state->requested_depth_format;
    state->msg.depth.depth_data_format = state->requested_depth_format;

    freenect_frame_mode vmode = freenect_find_video_mode(state->current_resolution, vfmt);
    freenect_frame_mode dmode = freenect_find_depth_mode(state->current_resolution, dfmt);

    freenect_set_video_mode(state->f_dev, vmode);
    freenect_set_depth_mode(state->f_dev, dmode);
}

void
cmd_cb(const lcm_recv_buf_t *rbuf __attribute__((unused)),
       const char *channel __attribute__((unused)),
       const kinect_cmd_msg_t *msg,
       void *user)
{
    state_t *self = (state_t *) user;

    if (msg->command_type == KINECT_CMD_MSG_T_SET_TILT) {
        dbg("Received tilt command; Angle : %d\n", msg->tilt_degree);
        self->freenect_angle = msg->tilt_degree;

        if (self->freenect_angle > 30)
            self->freenect_angle = 30;
        else if (self->freenect_angle < -20)
            self->freenect_angle = -20;

        // XXX is libfreenect thread-safe?
        freenect_set_tilt_degs(self->f_dev, self->freenect_angle);
    }
    else if (msg->command_type == KINECT_CMD_MSG_T_SET_LED) {
        // check that requested LED status is valid
        if (msg->led_status >= 0 && msg->led_status <= 6) {
            // XXX is libfreenect thread-safe?
            freenect_set_led(self->f_dev, msg->led_status);
        }
    }
    else if (msg->command_type == KINECT_CMD_MSG_T_SET_DEPTH_DATA_FORMAT) {
        // check that requested depth format is valid
        if (msg->depth_data_format >= 0 && msg->depth_data_format < 4) {
            self->requested_depth_format = msg->depth_data_format;
        }
    }
    else if (msg->command_type == KINECT_CMD_MSG_T_SET_IMAGE_DATA_FORMAT) {
        // TODO
    }
}

// Unpack buffer of (vw bit) data into padded 16bit buffer.
static inline void
convert_packed_to_16bit(uint8_t *raw, uint16_t *frame, int vw, int len)
{
    int mask = (1 << vw) - 1;
    uint32_t buffer = 0;
    int bitsIn = 0;
    while (len--) {
        while (bitsIn < vw) {
            buffer = (buffer << 8) | *(raw++);
            bitsIn += 8;
        }
        bitsIn -= vw;
        *(frame++) = (buffer >> bitsIn) & mask;
    }
}

void
depth_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
    state_t* state = (state_t*) freenect_get_user(dev);
    int64_t host_utime = timestamp_now();
    int64_t msg_utime = timestamp_sync(state->clocksync, timestamp, host_utime);

#if 0
    if(state->last_timestamp) {
        printf("D  depth: %10.6f  img  : %10.6f  last : %10.6f  (%"PRId64")\n",
               (msg_utime - state->last_depth_timestamp) * 1e-6,
               (msg_utime - state->last_img_timestamp) * 1e-6,
               (msg_utime - state->last_timestamp) * 1e-6,
               msg_utime);
    }
    state->last_depth_timestamp = msg_utime;
    state->last_timestamp = msg_utime;
#endif

    // don't capture a depth image until we have a camera image.  Assumes that
    // depth images always follow camera images in the data stream.
    if (!state->got_img)
        return;
    state->got_depth = 1;

    if (state->skip_depth) {
        state->msg.depth.timestamp = 0;
        state->msg.depth.width = 0;
        state->msg.depth.height = 0;
        state->msg.depth.depth_data_nbytes = 0;
        state->msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_NONE;
        state->msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
        state->msg.depth.uncompressed_size = 0;
        return;
    }

    state->msg.depth.timestamp = msg_utime;

    freenect_frame_mode md = freenect_find_depth_mode(state->current_resolution, state->current_depth_format);
    state->msg.depth.depth_data_nbytes = md.bytes;
    state->msg.depth.depth_data_format = state->current_depth_format;
    if (state->current_depth_format == KINECT_DEPTH_MSG_T_DEPTH_10BIT_PACKED
        || state->current_depth_format == KINECT_DEPTH_MSG_T_DEPTH_11BIT_PACKED) {
        assert(state->msg.depth.depth_data_nbytes <= state->depth_unpack_buf_size);
        convert_packed_to_16bit(data, state->depth_unpack_buf, md.data_bits_per_pixel, 640 * 480); //todo size shouldn't be hardcoded
    }
    else {
        assert(state->msg.depth.depth_data_nbytes <= state->depth_unpack_buf_size);
        memcpy(state->depth_unpack_buf, data, state->msg.depth.depth_data_nbytes);
    }

    //  switch(state->current_depth_format) {
    //      case KINECT_DEPTH_MSG_T_DEPTH_11BIT:
    //     state->msg.depth.depth_data_nbytes = FREENECT_DEPTH_11BIT_SIZE;
    //     state->msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_11BIT;
    //                      convert_packed_to_16bit(data, state->depth_unpack_buf, 11, FREENECT_FRAME_PIX);
    //     break;
    //   case KINECT_DEPTH_MSG_T_DEPTH_10BIT:
    //     state->msg.depth.depth_data_nbytes = FREENECT_DEPTH_10BIT_SIZE;
    //     state->msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_10BIT;
    //                      convert_packed_to_16bit(data, state->depth_unpack_buf, 10, FREENECT_FRAME_PIX);
    //     break;
    //  #if 0
    //   case FREENECT_DEPTH_11BIT_PACKED:
    //     state->msg.depth.depth_data_nbytes = FREENECT_DEPTH_11BIT_PACKED_SIZE;
    //     state->msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_11BIT_PACKED;
    //     break;
    //   case FREENECT_DEPTH_10BIT_PACKED:
    //     state->msg.depth.depth_data_nbytes = FREENECT_DEPTH_10BIT_PACKED_SIZE;
    //     state->msg.depth.depth_data_format = KINECT_DEPTH_MSG_T_DEPTH_10BIT_PACKED;
    //     break;
    //  #endif
    //   default:
    //     state->msg.depth.depth_data_nbytes = 0;
    //     state->msg.depth.depth_data_format = 0; // TODO spew warning
    //  }

    if (state->use_zlib) {
        int uncompressed_size = state->msg.depth.depth_data_nbytes;
        unsigned long compressed_size = state->depth_compress_buf_size;
        compress2(state->depth_compress_buf, &compressed_size, (void*) state->depth_unpack_buf, uncompressed_size,
                  Z_BEST_SPEED);
        state->msg.depth.depth_data_nbytes = (int) compressed_size;
        state->msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB;
        state->msg.depth.uncompressed_size = uncompressed_size;
    }
    else {
        assert(state->msg.depth.depth_data_nbytes <= state->depth_compress_buf_size);
        memcpy(state->depth_compress_buf, state->depth_unpack_buf, state->msg.depth.depth_data_nbytes);
        state->msg.depth.compression = KINECT_DEPTH_MSG_T_COMPRESSION_NONE;
        state->msg.depth.uncompressed_size = state->msg.depth.depth_data_nbytes;
    }
}

void
image_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
    state_t* state = (state_t*) freenect_get_user(dev);
    int64_t host_utime = timestamp_now();
    int64_t msg_utime = timestamp_sync(state->clocksync, timestamp, host_utime);

#if 0
    if(state->last_timestamp) {
        printf(" I depth: %10.6f  img  : %10.6f  last : %10.6f  (%"PRId64")\n",
               (msg_utime - state->last_depth_timestamp) * 1e-6,
               (msg_utime - state->last_img_timestamp) * 1e-6,
               (msg_utime - state->last_timestamp) * 1e-6,
               msg_utime);
    }
    state->last_img_timestamp = msg_utime;
    state->last_timestamp = msg_utime;
#endif

    if (!rate_check(state->capture_rate)) {
        return;
    }
    state->got_img = 1;

    // if we don't actually care about images, then don't process it.
    if (state->skip_img) {
        state->msg.image.timestamp = 0;
        state->msg.image.width = 0;
        state->msg.image.height = 0;
        state->msg.image.image_data_nbytes = 0;
        state->msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_NONE;
        return;
    }

    state->msg.image.timestamp = msg_utime;

    // Do we need to de-Bayer the image?
    if (state->current_image_format == KINECT_IMAGE_MSG_T_VIDEO_BAYER ||
        state->current_image_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {

        cam_pixel_convert_bayer_to_8u_bgra(state->debayer_buf, state->debayer_buf_stride,
                                           640, 480, data, 640, CAM_PIXEL_FORMAT_BAYER_GRBG);

        // convert from bgra -> rgb and place the result back into the original
        // bayer buffer
        cam_pixel_convert_8u_bgra_to_8u_rgb((uint8_t*) data, 640 * 3, 640, 480,
                                            state->debayer_buf, state->debayer_buf_stride);
        state->msg.image.image_data_nbytes = 640 * 480 * 3;
        state->msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_RGB;
    }

    // do we need to JPEG compress the image?
    if (state->current_image_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
        int compressed_size = state->image_buf_size;
#if USE_JPEG_UTILS_POD
        int compression_status = jpeg_compress_8u_rgb (data, 640, 480, 640*3,
                                                       state->image_buf, &compressed_size, state->jpeg_quality);
#else
        int compression_status = jpegijg_compress_8u_rgb(data, 640, 480, 640 * 3,
                                                         state->image_buf, &compressed_size, state->jpeg_quality);
#endif

        if (0 != compression_status) {
            fprintf(stderr, "JPEG compression failed...\n");
        }
        state->msg.image.image_data_nbytes = compressed_size;
        state->msg.image.image_data_format = KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG;

    }
    else {
        assert(state->msg.image.image_data_nbytes <= state->image_buf_size);
        memcpy(state->image_buf, data, state->msg.image.image_data_nbytes);
    }
}

//int freenect_write_register(freenect_device *dev, uint16_t reg, uint16_t data);

static void *
freenect_threadfunc(void *user_data)
{
    printf("starting kinect thread...\n");
    state_t* state = (state_t*) user_data;

    freenect_set_tilt_degs(state->f_dev, state->freenect_angle);
    freenect_set_led(state->f_dev, state->current_led);
    freenect_set_depth_callback(state->f_dev, depth_cb);
    freenect_set_video_callback(state->f_dev, image_cb);
    set_image_depth_formats(state);
    freenect_set_video_buffer(state->f_dev, state->image_data);

    //  if(state->capture_rate->target_hz <= 15) {
    //    int rval = (int)ceil(state->capture_rate->target_hz);
    //    int st = freenect_write_register(state->f_dev, 0xE, rval);
    //    printf("Decreasing RGB framerate to %d: %d\n", rval, st);
    //    st = freenect_write_register(state->f_dev, 0x14, rval);
    //    printf("Decreasing depth framerate to %d: %d\n", rval, st);
    //  }

    freenect_start_depth(state->f_dev);
    freenect_start_video(state->f_dev);

    while (!state->die && freenect_process_events(state->f_ctx) >= 0) {

        // ready to publish?
        if (state->got_img && state->got_depth) { // XXX could transmit sooner if we don't want to publish depth data
            populate_status(state, &state->msg, timestamp_now());
            kinect_frame_msg_t_publish(state->lcm, state->msg_channel, &state->msg);

            state->got_img = 0;
            state->got_depth = 0;
        }

#if 0
        // do we need to change video formats?
        if (state->requested_image_format != state->current_image_format) {
            dbg("Changing Image format\n");
            freenect_stop_video(state->f_dev);
            freenect_set_video_format(state->f_dev, state->requested_image_format);
            freenect_start_video(state->f_dev);
            state->current_image_format = state->requested_image_format;
        }

        // do we need to change depth formats?
        if (state->requested_depth_format != state->current_depth_format) {
            dbg("Changing Depth format\n");
            freenect_stop_depth(state->f_dev);
            freenect_set_depth_format(state->f_dev, state->requested_depth_format);
            freenect_start_depth(state->f_dev);
            state->current_depth_format = state->requested_depth_format;
        }
#endif

        if (rate_check(state->report_rate)) {
            printf("Capture rate: %5.2fHz (%6"PRId64")\n", state->capture_rate->current_hz, state->capture_rate->tick_count);
        }
    }

    printf("\nshutting down streams...\n");

    freenect_stop_depth(state->f_dev);
    freenect_stop_video(state->f_dev);

    freenect_close_device(state->f_dev);
    freenect_shutdown(state->f_ctx);

    printf("-- done!\n");
    return NULL;
}

//pthread - for publishing sensor status 
static void *status_thread(void *user)
{
    state_t *self = (state_t*) user;

    while(1){
        if(self->report_rate){
            kinect_sensor_status_t msg;
            msg.utime = timestamp_now();
            msg.sensor_name = "kinect"; //maybe use some indexing - to make it unique
            msg.rate = self->capture_rate->current_hz;
            
            msg.type = KINECT_SENSOR_STATUS_T_KINECT; //prob need to identify this more - if there are multiple kinects

            kinect_sensor_status_t_publish(self->lcm, "SENSOR_STATUS_KINECT", &msg);
        }
        sleep(1);
    }
    
    return 0;
}

static void usage(const char* progname)
{
    fprintf(stderr, "Usage: %s [options]\n"
            "\n"
            "Options:\n"
            "  -r RATE   Throttle publishing to RATE Hz.\n"
            "  -d        Depth mode\n"
            "  -i        Image mode\n"
            "  -j        JPEG-compress RGB images\n"
            "  -q QUAL   JPEG compression quality (0-100, default 94)\n"
            "  -z        ZLib compress depth images\n"
            "  -l URL    Specify LCM URL\n"
            "  -h        This help message\n"
            "  -n dev    Number of the device to open\n"
            "  -c name   LCM channel\n",
            g_path_get_basename(progname));

    fprintf(stderr, "Image mode must be one of:\n"
            "  VIDEO_RGB             = 0\n"
            "  VIDEO_BAYER           = 1\n"
            "  VIDEO_IR_8BIT         = 2\n"
            "  VIDEO_IR_10BIT        = 3\n"
            "  VIDEO_IR_10BIT_PACKED = 4\n"
            "  VIDEO_YUV_RGB         = 5\n"
            "  VIDEO_YUV_RAW         = 6\n"
            "\n"
            "  VIDEO_DISABLED        = -1\n"
            );

    fprintf(stderr, "Depth mode must be one of:\n"
            "  DEPTH_11BIT        = 0\n"
            "  DEPTH_10BIT        = 1\n"
            "  DEPTH_11BIT_PACKED = 2\n"
            "  DEPTH_10BIT_PACKED = 3\n"
            "  DEPTH_REGISTERED   = 4\n"
            "  DEPTH_MM           = 5\n"
            "\n"
            "  DEPTH_DISABLED         =-1\n"
            );
    exit(1);
}

int main(int argc, char **argv)
{
    state_t *state = (state_t*) calloc(1, sizeof(state_t));

    double target_rate = INFINITY;

    state->skip_img = 0;
    state->skip_depth = 0;
    state->throttle = 0;
    state->capture_rate = NULL;

    state->freenect_thread = NULL;
    state->die = 0;

    state->f_ctx = NULL;
    state->f_dev = NULL;
    state->freenect_angle = 0;
    state->freenect_led = 0;

    state->jpeg_quality = 94;

    // make these configurable - either/both from the command line and from outside LCM command
    state->requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_BAYER;
    state->requested_depth_format = KINECT_DEPTH_MSG_T_DEPTH_11BIT;
    state->requested_led = LED_RED;
    state->current_image_format = state->requested_image_format;
    state->current_depth_format = state->requested_depth_format;
    state->current_led = state->requested_led;
    int user_device_number = 0;
    state->msg_channel = g_strdup("KINECT_FRAME");

    //todo this shouldn't be hardcoded
    state->current_resolution = FREENECT_RESOLUTION_MEDIUM;

    int c;
    char *lcm_url = NULL;
    // command line options - to throtle - to ignore image publish  
    while ((c = getopt(argc, argv, "hd:i:r:jq:zl:n:c:")) >= 0) {
        switch (c) {
        case 'i': //ignore images
            state->requested_image_format = atoi(optarg);
            break;
        case 'd':
            state->requested_depth_format = atoi(optarg);
            break;
        case 'j':
            state->requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG;
            break;
        case 'q':
            state->jpeg_quality = atoi(optarg);
            if (state->jpeg_quality < 0 || state->jpeg_quality > 100)
                usage(argv[0]);
            break;
        case 'n':
            user_device_number = atoi(optarg);
            printf("attempting to open device %i\n", user_device_number);
            break;
        case 'z':
            state->use_zlib = 1;
            printf("ZLib compressing depth data\n");
            break;
        case 'r':
            target_rate = strtod(optarg, NULL);
            printf("Target Rate is : %.3f Hz\n", target_rate);
            state->throttle = 1;
            break;
        case 'l':
            lcm_url = strdup(optarg);
            printf("Using LCM URL \"%s\"\n", lcm_url);
            break;
        case 'c':
            g_free(state->msg_channel);
            state->msg_channel = g_strdup(optarg);
            printf("Output on LCM channel: %s\n", state->msg_channel);
            break;
        case 'h':
        case '?':
            usage(argv[0]);
        }
    }

    const char * depthModeSting[] = {
        "DEPTH_11BIT",
        "DEPTH_10BIT",
        "DEPTH_11BIT_PACKED",
        "DEPTH_10BIT_PACKED",
        "DEPTH_REGISTERED",
        "DEPTH_MM",
    };
    switch (state->requested_depth_format) {
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT:
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT:
    case KINECT_DEPTH_MSG_T_DEPTH_11BIT_PACKED:
    case KINECT_DEPTH_MSG_T_DEPTH_10BIT_PACKED:
    case KINECT_DEPTH_MSG_T_DEPTH_REGISTERED:
    case KINECT_DEPTH_MSG_T_DEPTH_MM:
        dbg("Depth Mode is %s\n", depthModeSting[state->requested_depth_format]);
        break;
    case KINECT_DEPTH_MSG_T_DEPTH_NONE:
    case -1:
        dbg("Depth is disabled");
        state->requested_depth_format = KINECT_DEPTH_MSG_T_DEPTH_NONE;
        state->skip_depth = 1;
        break;
    default:
        dbg("Invalid depth format %d\n", state->requested_depth_format);
        usage(argv[0]);
        break;
    }

    const char * imageModeSting[] = {
        "VIDEO_RGB",
        "VIDEO_BAYER",
        "VIDEO_IR_8BIT",
        "VIDEO_IR_10BIT",
        "VIDEO_IR_10BIT_PACKED",
        "VIDEO_YUV_RGB",
        "VIDEO_YUV_RAW",
    };

    switch (state->requested_image_format) {
    case KINECT_IMAGE_MSG_T_VIDEO_RGB:
    case KINECT_IMAGE_MSG_T_VIDEO_BAYER:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_8BIT:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT:
    case KINECT_IMAGE_MSG_T_VIDEO_IR_10BIT_PACKED:
    case KINECT_IMAGE_MSG_T_VIDEO_YUV_RGB:
    case KINECT_IMAGE_MSG_T_VIDEO_YUV_RAW:
        dbg("Image Mode is %s\n", imageModeSting[state->requested_image_format]);
        break;

    case KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG:
        dbg("Jpeg Compressing RGB images\n");
        break;
    case KINECT_IMAGE_MSG_T_VIDEO_NONE:
    case -1:
        dbg("Image is disabled");
        state->requested_image_format = KINECT_IMAGE_MSG_T_VIDEO_NONE;
        state->skip_img = 1;
        break;
    default:
        dbg("Invalid image format %d\n", state->requested_image_format);
        usage(argv[0]);
        break;
    }

    /*
      int user_device_number = 0;
      if (argc > 1) {
      user_device_number = atoi(argv[1]);
      printf("attempting to open device %i\n", user_device_number);
      }
    */

    // throttling
    state->capture_rate = rate_new(target_rate);

    //todo the width and height should be computed from the desired resolution

    // allocate image and depth buffers
    state->image_data = (uint8_t*) malloc(640 * 480 * 4);

    // allocate more space for the image buffer, as we might use it for compressed data
    state->image_buf_size = 640 * 480 * 10;
    if (0 != posix_memalign((void**) &state->image_buf, 16, state->image_buf_size)) {
        fprintf(stderr, "Error allocating image buffer\n");
        return 1;
    }
    state->msg.image.image_data = state->image_buf;
    state->msg.image.width = 640; //TODO: this shouldn't be hardcoded
    state->msg.image.height = 480;

    // allocate a buffer for bayer de-mosaicing
    state->debayer_buf_size = 640 * 480 * 4;
    state->debayer_buf_stride = 640 * 4;
    if (0 != posix_memalign((void**) &state->debayer_buf, 16, state->debayer_buf_size)) {
        fprintf(stderr, "error allocating de-Bayer buffer\n");
        return 1;
    }

    // allocate space for unpacking depth data
    state->depth_unpack_buf_size = 640 * 480 * sizeof(uint16_t);
    state->depth_unpack_buf = (uint16_t*) malloc(state->depth_unpack_buf_size);

    // allocate space for zlib compressing depth data
    state->depth_compress_buf_size = 640 * 480 * sizeof(int16_t) * 4;
    state->depth_compress_buf = (uint8_t*) malloc(state->depth_compress_buf_size);
    state->msg.depth.depth_data = state->depth_compress_buf;
    state->msg.depth.width = 640;
    state->msg.depth.height = 480;

    state->got_img = 0;
    state->got_depth = 0;

    state->report_rate = rate_new(0.5);

    state->last_timestamp = 0;

    // initialize LCM

    state->lcm = lcm_create(lcm_url);

    if (!state->lcm) {
        fprintf(stderr, "Unable to initialize LCM\n");
        return 1;
    }

    // initialize the kinect device
    if (freenect_init(&state->f_ctx, NULL) < 0) {
        printf("freenect_init() failed\n");
        return 1;
    }

    freenect_set_log_level(state->f_ctx, FREENECT_LOG_INFO);

    int num_devices = freenect_num_devices(state->f_ctx);
    printf("Number of devices found: %d\n", num_devices);

    if (num_devices < 1)
        return 1;

    if (freenect_open_device(state->f_ctx, &state->f_dev, user_device_number) < 0) {
        printf("Could not open device\n");
        return 1;
    }

    freenect_set_user(state->f_dev, state);

    // setup passive time synchronization so we can guess the true image
    // acquisition times
    state->clocksync = timestamp_sync_init(100000000, 0xFFFFFFFFLL, 1.001);

    if (!g_thread_supported())
        g_thread_init(NULL);

    // subscribe to kinect command messages
    kinect_cmd_msg_t_subscribe(state->lcm, "KINECT_CMD", cmd_cb, state);

    GError *thread_err = NULL;
    state->freenect_thread = g_thread_create(freenect_threadfunc, state, TRUE, &thread_err);
    if (thread_err) {
        fprintf(stderr, "Error creating thread: %s\n", thread_err->message);
        return 1;
    }

    pthread_create(&state->work_thread, NULL, status_thread, state);

    while (!state->die) {
        lcm_handle(state->lcm);
    }

    timestamp_sync_free(state->clocksync);
    g_free(state->msg_channel);
    rate_destroy(state->capture_rate);
    free(state);

    return 0;
}
