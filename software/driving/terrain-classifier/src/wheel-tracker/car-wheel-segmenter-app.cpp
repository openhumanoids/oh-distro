#define WINDOW_NAME "Car wheel segmenter"

#include <set>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <opencv2/opencv.hpp>
#include <lcmtypes/drc_driving_wheel_state_t.h>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

int MAX_IMAGE_WIDTH = 0;
int MAX_IMAGE_HEIGHT = 0;

using namespace cv;

struct state_t {
    lcm_t *lcm;
    pthread_t lcm_thread;
    pthread_mutex_t img_mutex;

    GMainLoop *mainloop;
    BotParam *param;
    BotFrames *frames;
    int counter;

    // Img
    cv::Mat img; 

    // utimes for image
    int64_t  img_utime;

    // UI selection of desired object
    Point origin, destination;
    float theta_off; 
    bool selectObject;

    drc_driving_wheel_state_t* tracking_result;

    state_t () {
        // LCM, BotFrames, BotParam inits
        lcm =  bot_lcm_get_global(NULL);
        param = bot_param_new_from_server(lcm, 1);
        frames = bot_frames_get_global (lcm, param);

        img_mutex = PTHREAD_MUTEX_INITIALIZER;

        // Counter for debug prints
        counter = 0; 
        tracking_result = NULL;
        selectObject = false;
    }
    ~state_t () { 
    }

};
state_t * state = NULL;

void* lcm_thread_handler(void *l) {
    state_t* state = (state_t*)l;
    while(1)
        lcm_handle(state->lcm);

}
struct MouseEvent {
    MouseEvent() { event = -1; buttonState = 0; }
    Point pt;
    int event;
    int buttonState;
};
MouseEvent mouse;

static void onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseEvent* data = (MouseEvent*)userdata;

    if (!MAX_IMAGE_WIDTH || !MAX_IMAGE_HEIGHT)
        return;

    float sx = 1.f / MAX_IMAGE_WIDTH ; 
    float sy = 1.f / MAX_IMAGE_HEIGHT;
    if (state->selectObject) {
        state->destination = Point(x,y);
        cv::Point2f ovec(state->destination.y-state->origin.y, state->destination.x-state->origin.x);
        float norm = cv::norm(ovec);
        state->theta_off = 0; 
        if (norm != 0) { 
            ovec *= 1.f / norm;
            state->theta_off = atan2(ovec.y,ovec.x);
        }
    }

    switch (event) {
    case CV_EVENT_LBUTTONDOWN:
        state->origin = Point(x, y);
        state->destination = Point(x, y);
        state->selectObject = true;
        break;
    case CV_EVENT_LBUTTONUP:
        state->selectObject = false;
        drc_driving_wheel_state_t wheel_state;
        wheel_state.utime = state->img_utime;
        wheel_state.u = state->origin.x * sx;
        wheel_state.v = state->origin.y * sy;
        wheel_state.theta = state->theta_off;
        drc_driving_wheel_state_t_publish(state->lcm, "WHEEL_STATE_SEGMENT", &wheel_state);

        std::cerr << "SEND selection: " << state->img_utime << " - " << 
            wheel_state.u << " " << wheel_state.v << " " << 
            state->theta_off << std::endl;

        std::cerr << "or: " << state->origin << " dest: " << state->destination << " ang: " << state->theta_off << std::endl;

        break;
    }
    return;
}

void  INThandler(int sig)
{
    printf("Exiting\n");
    if (state) delete state; 
    exit(0);
}

void
decode_image(const bot_core_image_t * msg, cv::Mat& img)
{
    int ch = msg->row_stride / msg->width;
    if (img.empty() || img.rows != msg->height || img.cols != msg->width)
        img.create(msg->height, msg->width, CV_8UC3);

  // extract image data
  switch (msg->pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
        memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height * 3);
        cv::cvtColor(img, img, CV_RGB2BGR);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // for some reason msg->row_stride is 0, so we use msg->width instead.
        if (ch == 3) { 
            jpeg_decompress_8u_rgb(msg->data,
                                    msg->size,
                                    img.data,
                                    msg->width,
                                    msg->height,
                                    msg->width * ch);
            cv::cvtColor(img, img, CV_RGB2BGR);
        } else { 
            jpeg_decompress_8u_gray(msg->data,
                                    msg->size,
                                    img.data,
                                    msg->width,
                                    msg->height,
                                    msg->width);
        }
      break;
  case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      memcpy(img.data, msg->data, sizeof(uint8_t) * msg->width * msg->height);
      break;
    default:
        fprintf(stderr, "Unrecognized image format : %d\n", (int) msg->pixelformat);
      break;
  }
  return;
}

static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    if (!msg->width || !msg->height) return;
    if (!MAX_IMAGE_WIDTH || !MAX_IMAGE_HEIGHT) { 
        MAX_IMAGE_WIDTH = msg->width;
        MAX_IMAGE_HEIGHT = msg->height;
    }

    pthread_mutex_lock(&state->img_mutex);

    state_t* state = (state_t*) user_data; 
    decode_image(msg, state->img);    
    state->img_utime = msg->utime; 

    pthread_mutex_unlock(&state->img_mutex);
    return;
}

static void on_result_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                             const drc_driving_wheel_state_t* msg, 
                             void *user_data ) {
    pthread_mutex_lock(&state->img_mutex);
    state_t* state = (state_t*) user_data; 

    if(state->tracking_result != NULL){
        drc_driving_wheel_state_t_destroy(state->tracking_result);
    }
    state->tracking_result = drc_driving_wheel_state_t_copy(msg);
    pthread_mutex_unlock(&state->img_mutex);
    return;
}

struct CarWheelSegmenterOptions { 
    bool vDEBUG;
    std::string vCHANNEL;

    CarWheelSegmenterOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false) {}
};

CarWheelSegmenterOptions options;

int main(int argc, char** argv)
{
    std::cout << "============= QUICK MODES ===================\n";
    std::cout << "drc-car-wheel-segmenter -c CAMERALEFT\n";
    std::cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();

    std::cerr << "===========  Car wheel segmenter ============" << std::endl;
    std::cerr << "=> CAMERA CHANNEL : " << options.vCHANNEL << std::endl;
    std::cerr << "=> DEBUG : " << options.vDEBUG << std::endl;
    std::cerr << "===============================================" << std::endl;

    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    // Param server, botframes
    state = new state_t();

    printf("starting lcm thread\n");
    pthread_create(&(state->lcm_thread), NULL, lcm_thread_handler, state);

    cv::namedWindow( WINDOW_NAME );
    cv::setMouseCallback( WINDOW_NAME, onMouse, &mouse);

    // Subscriptions
    bot_core_image_t_subscribe(state->lcm, options.vCHANNEL.c_str(), on_image_frame, (void*)state);
    drc_driving_wheel_state_t_subscribe(state->lcm, "WHEEL_STATE_ESTIMATE", on_result_frame, (void*)state);

    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    while(1) { 
        unsigned char c = cv::waitKey(1) & 0xff;
        if (c == 'q') { 
            break;
        }
        pthread_mutex_lock(&state->img_mutex);

        // UI handling 
        if (!state->img.empty()) { 
            cv::Mat display;
            cv::resize(state->img.clone(), display, cv::Size(MAX_IMAGE_WIDTH,MAX_IMAGE_HEIGHT)); 

            if (state->selectObject) { 
                cv::circle(display, state->origin, 20, cv::Scalar(0,255,0), 2, CV_AA);
                cv::line(display, state->origin, state->destination, cv::Scalar(0,255,255), 2, CV_AA);
            }

            if (state->tracking_result && !state->selectObject) { 
                float sz = 150; 
                float cx = state->tracking_result->u * MAX_IMAGE_WIDTH;
                float cy = state->tracking_result->v * MAX_IMAGE_HEIGHT;
                float dx = cx + sin(state->theta_off - state->tracking_result->theta)*sz;
                float dy = cy + cos(state->theta_off - state->tracking_result->theta)*sz;
                cv::circle(display, cv::Point(cx,cy), 20, cv::Scalar(0,255,0), 2, CV_AA);
                cv::line(display, cv::Point(cx,cy), cv::Point(dx,dy), cv::Scalar(0,255,255), 2, CV_AA);
            }

            imshow(WINDOW_NAME, display);
        }
        pthread_mutex_unlock(&state->img_mutex);
    }
    return 0;
}
