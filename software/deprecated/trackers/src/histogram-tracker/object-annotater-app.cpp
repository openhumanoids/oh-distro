#define WINDOW_NAME "Object Annotater"

#include <set>
#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <opencv2/opencv.hpp>
#include <lcmtypes/drc_point_list_t.h>

#include <trackers/object-segmenter.hpp>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

using namespace cv;

const int SEG_SIZE = 10;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;

struct state_t {
    lcm_t *lcm;
    pthread_t lcm_thread;
    pthread_mutex_t img_mutex;

    GMainLoop *mainloop;
    BotParam *param;

    int counter;

    // Img
    cv::Mat img, mask3; 

    ObjectSegmenter segmenter;
    bool segmented; 

    // utimes for image
    int64_t  img_utime;

    std::vector<cv::Point> fg_pts;

    state_t () {
        // LCM, BotFrames, BotParam inits
        lcm =  bot_lcm_get_global(NULL);
        param = bot_param_new_from_server(lcm, 1);

        img_mutex = PTHREAD_MUTEX_INITIALIZER;

        // Counter for debug prints
        counter = 0; 
        segmented = false;
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
    cv::Point pt;
    int event;
    int buttonState;
};
MouseEvent mouse;

static void onMouse(int event, int x, int y, int flags, void* userdata) {
    MouseEvent* data = (MouseEvent*)userdata;

    switch (event) {
    case CV_EVENT_LBUTTONDOWN:
        break;
    case CV_EVENT_LBUTTONUP:
        state->segmented = false;
        state->fg_pts.push_back(cv::Point(x,y));

        // Initialize and segment image
        state->segmenter.initialize(state->img, state->fg_pts);
        cv::Mat1b object_mask = state->segmenter.segment(state->img);
        
        cv::cvtColor(object_mask, state->mask3, CV_GRAY2BGR);
        state->segmented = true;
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
    int ch = msg->row_stride / (msg->width); 
    if (ch != 3) { 
        std::cerr << " ERROR: REQUIRE COLOR IMAGES. GrayScale Image received!" << std::endl;
        return;
    }
    assert(ch == 3);
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
        jpeg_decompress_8u_rgb(msg->data,
                               msg->size,
                               img.data,
                               msg->width,
                               msg->height,
                               msg->width * ch);
            cv::cvtColor(img, img, CV_RGB2BGR);
        break;
    default:
        fprintf(stderr, "Unrecognized image format\n");
        break;
    }
    return;
}

static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    if (!msg->width || !msg->height) return;
    
    // While Annotation, do not update frame
    if (state->fg_pts.size()) return;

    pthread_mutex_lock(&state->img_mutex);

    state_t* state = (state_t*) user_data; 
    decode_image(msg, state->img);    
    state->img_utime = msg->utime; 
    std::cerr << "tx: " << msg->utime << std::endl;
    cv::resize(state->img, state->img, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT));

    pthread_mutex_unlock(&state->img_mutex);
    return;
}

// static void on_result_frame (const lcm_recv_buf_t *rbuf, const char *channel,
//                              const drc_driving_wheel_state_t* msg, 
//                              void *user_data ) {
//     pthread_mutex_lock(&state->img_mutex);
//     state_t* state = (state_t*) user_data; 

//     if(state->tracking_result != NULL){
//         drc_driving_wheel_state_t_destroy(state->tracking_result);
//     }
//     state->tracking_result = drc_driving_wheel_state_t_copy(msg);
//     pthread_mutex_unlock(&state->img_mutex);
//     return;
// }

struct ObjectAnnotaterOptions { 
    bool vDEBUG;
    std::string vCHANNEL;

    ObjectAnnotaterOptions () : 
        vCHANNEL(std::string("CAMERALEFT")), vDEBUG(false) {}
};

ObjectAnnotaterOptions options;

int main(int argc, char** argv)
{
    std::cout << "============= QUICK MODES ===================\n";
    std::cout << "drc-object-annotater -c CAMERALEFT\n";
    std::cout << "=============================================\n";

    ConciseArgs opt(argc, (char**)argv);
    opt.add(options.vCHANNEL, "c", "camera-channel","Camera Channel [CAMERALEFT]");
    opt.add(options.vDEBUG, "d", "debug","Debug mode");
    opt.parse();

    std::cerr << "===========  object-annotater ============" << std::endl;
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

    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    while(1) { 
        unsigned char c = cv::waitKey(1) & 0xff;
        if (c == 'q') { 
            break;
        } else if ( c == 'r' ) { 
            state->fg_pts.clear(); 
            state->segmented = false;
        } else if ( c == 's' ) { 
            // Send annotation points 
            drc_point_list_t pts_list;
            pts_list.utime = state->img_utime;
            pts_list.n_points = state->fg_pts.size(); 
            pts_list.uv = (int16_t**) malloc(sizeof(int16_t*) * pts_list.n_points);
            for (int j=0; j<state->fg_pts.size(); j++) { 
                std::cerr << "pts: " << state->fg_pts[j].x << " " << state->fg_pts[j].y << std::endl;
                int16_t x = (int16_t)state->fg_pts[j].x;
                int16_t y = (int16_t)state->fg_pts[j].y;
                pts_list.uv[j] = (int16_t*) malloc(sizeof(int16_t) * 2);
                pts_list.uv[j][0] = x, pts_list.uv[j][1] = y;
            }
            drc_point_list_t_publish(state->lcm, "OBJECT_SEGMENTATION_ANNOTATION", &pts_list);
            std::cerr << "SENDING utime: " << state->img_utime << std::endl;        

            for (int j=0; j<state->fg_pts.size(); j++)
                free(pts_list.uv[j]);
            free(pts_list.uv);

        }

        pthread_mutex_lock(&state->img_mutex);
        if (!state->img.empty()) { 
            cv::Mat display = state->img.clone();
            for (int j=0; j<state->fg_pts.size(); j++) {
                cv::Point tl(state->fg_pts[j].x-SEG_SIZE/2, state->fg_pts[j].y-SEG_SIZE/2);
                cv::Point br(state->fg_pts[j].x+SEG_SIZE/2, state->fg_pts[j].y+SEG_SIZE/2);
                // cv::rectangle(display, tl, br, cv::Scalar(0,255,0), 1, CV_AA);
                cv::circle(display, state->fg_pts[j], 5, cv::Scalar(0,255,0), 1, CV_AA);
            }

            if (state->segmented && !state->mask3.empty()) { 
                display = 0.6 * state->mask3 + 0.4 * display;
            }
            imshow(WINDOW_NAME, display);
        }
        pthread_mutex_unlock(&state->img_mutex);
    }
    return 0;
}
