/*
TLD works well w/ features (keypts) low-level features


Multi-modal fusion of observations


- DoT works well with textureless objects (contoures)
- Use both to bootstrap a contour and features based reconstruction 
- Does going to 3D from contour make tracking possible ??
*/
#include "multi_object_tracker.hpp"

using namespace cv;
// using namespace spvision;
static std::ostream& operator << (std::ostream& os, const cv::Rect& r) {
    return (os << "[x=" << r.x << ", y=" << r.y << ", w=" << r.width << ", h=" << r.height << "]");
}

static void usage(const char* progname)
{
  fprintf (stderr, "Usage: %s [options]\n"
                   "\n"
                   "Options:\n"
                   "  -l URL    Specify LCM URL\n"
                   "  -h        This help message\n", 
                   g_path_get_basename(progname));
  exit(1);
}

void  INThandler(int sig) {
    printf("Exiting\n");
    exit(0);
}


// double velodyne_depth(cv::Rect& rect) { 
//     if (!depth_buffer.size()) { 
//         std::cerr << "SEGMENTATION RETURNED null depth due to lack of depth buffer" << std::endl;
//         return 0; 
//     }

//     cv::Mat depth = depth_buffer.front(); 
//     float* pdepth = (float*)depth.data; 

//     // std::vector<cv::Point2f> pts; 
//     std::vector<double> depths; 
//     for (int y=rect.y; y<rect.y+rect.height && y < depth.rows; y++) { 
//         for (int x=rect.x; x<rect.x+rect.width && x < depth.cols; x++) { 
//             float d = pdepth[y*depth.cols + x];
//             if (d >= 0.1f) { 
//                 depths.push_back(d); 
//                 // pts.push_back(cv::Point2f(x,y)); 
//             }
//         }
//     }
//     std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end()); 
//     return depths[depths.size()/2];
// }

double stereo_bm_depth(cv::Rect& rect, int64_t utime) { 
    if (!cbuffer.size()) { 
        std::cerr << "SEGMENTATION RETURNED null depth due to lack of depth buffer" << std::endl;
        return 0; 
    }

    cv::Mat depth;
    // std::cerr << "INIT UTIME: " << tinfo->init_utime * 1e-6 << std::endl;
    for (int j=0; j<cbuffer.size(); j++) { 
        // either match or last one
        // std::cerr << "depth UTIME: " << utime << " " << cbuffer[j].first << std::endl;
        if (cbuffer[j].first == utime || j == cbuffer.size() - 1 ) { 
            depth = cbuffer[j].second.second.clone();
        }
    }
    float* pdepth = (float*)(depth.data); 

    // std::vector<cv::Point2f> pts; 
    std::vector<float> depths; 
    for (int y=rect.y; y<rect.y+rect.height && y < depth.rows; y++) { 
        for (int x=rect.x; x<rect.x+rect.width && x < depth.cols; x++) { 
            float d = pdepth[y*depth.cols + x];
            if (d >= 0.1f) { 
                depths.push_back(d); 
            }
        }
    }
    if (!depths.size()) return 0; 
    //std::cerr << "DEPTHS: " << cv::Mat(depths) << std::endl;

    
    int bins = 30;
    int histSize[] = {bins};
    float lab_ranges[] = { 0.f, 100.f };
    const float* ranges[] = { lab_ranges };
    int channels[] = {0};
    
    cv::Mat_<float> data(depths.size(), 1, &depths[0]);
    cv::Mat hist;
    cv::calcHist( &data, 1, channels, cv::Mat(), // do not use mask
                      hist, 1, histSize, ranges,
                      true, // the histogram is uniform
                      false );
    cv::normalize(hist, hist, 0, 1, CV_MINMAX);
    if (vDEBUG)
        std::cerr << "HIST: " << hist << std::endl;

    // First mode when approaching from min depth to max depth
    int idx = 0; 
    for (; idx<bins; idx++)
        if (hist.at<float>(idx) >= 0.5)
            break;

    if (idx == bins -1) { 
        std::nth_element(depths.begin(), depths.begin() + depths.size()/2, depths.end()); 
        return depths[depths.size()/2];
    } else {
        float min_val = 1.f * idx * (lab_ranges[1] - lab_ranges[0]) / (1.f * bins) ;
        float max_val = 1.f * (idx + 1) * (lab_ranges[1] - lab_ranges[0]) / (1.f * bins) ;
        if (vDEBUG)
            std::cerr << "BETWEEN: " << min_val << " and " << max_val << std::endl;
        std::vector<float> depths_bin;
        for (int j=0; j<depths.size(); j++) 
            if (depths[j] >= min_val && depths[j] <= max_val)
                depths_bin.push_back(depths[j]);
        std::nth_element(depths_bin.begin(), depths_bin.begin() + depths_bin.size()/2, depths_bin.end()); 
        return depths_bin[depths_bin.size()/2];
    }

    // std::cerr << "dist: " << cv::Mat(depths) << std::endl;




}




void select_object(int64_t utime, int32_t object_id, int32_t feature_id, cv::Rect& roi) { 

    // tld tracker selection
    if (roi.height < vMIN_DIM || roi.width < vMIN_DIM || roi.height > 400 || roi.width > 400) { 
        std::cerr << "Init FAILED: ROI[(" 
                  << roi.x << "," << roi.y 
                  <<"), w="<< roi.width 
                  <<", h="<< roi.height << std::endl; 
        std::cerr << "Re-try segmenting with a bigger ROI" << std::endl;
        return;
    }

    std::cerr << "SELECTING OBJECT: " << roi << " " << cbuffer.size() << " " << utime << std::endl;

    //pthread_mutex_lock(&buffer_mutex);
    for (int j=cbuffer.size()-1; j>0; j--) { 
        std::cerr << "Searching : " << cbuffer[j-1].first << " " << cbuffer[j].first << std::endl;
        if ((cbuffer[j-1].first >= utime && cbuffer[j].first <= utime) || j==1) { 
            std::cerr << "Init TLD with " << cbuffer[j-1].first << std::endl;
	    
            const image_pair& stereo = cbuffer[j].second; 
            
            Mat gray; 
	    Mat img = stereo.first.clone(); // assume segmentation is from left
            if (img.channels() == 1)
                gray = img; 
            else 
                cvtColor(img, gray, CV_BGR2GRAY);
            
            std::pair<int32_t, int32_t> sel_hash = std::make_pair(object_id, feature_id); 
            if (tld_tracker_map.find(sel_hash)
                != tld_tracker_map.end()) { 
                std::cerr << "ReInitializing OBJ: " << object_id 
                          << "FEAT: " << feature_id << std::endl;
                tld_tracker_map.erase(sel_hash); 
            } else { 
                std::cerr << "Initializing OBJ: " << object_id 
                          << "FEAT: " << feature_id << std::endl;
            }

            // Check if within image
            if (!(roi.x >= 0 && roi.y >=0 && roi.x + roi.width <= gray.cols && roi.y + roi.height <= gray.rows))

            sel_hash.second = 0; 
            std::cerr << esc_green << "TLD Initialized! " << sel_hash.first << esc_def << std::endl;
            TLDInfo* tinfo = new TLDInfo(utime, sel_hash.first, sel_hash.second); // camera left
            cv::Rect selection(roi.x, roi.y, roi.width, roi.height);

            std::cerr << "selection: " << selection << std::endl;
            tinfo->tld->selectObject(gray, &selection);
            tld_tracker_map.insert(std::make_pair(sel_hash, tinfo)); 

	    rectangle(img, selection, CV_RGB(255, 0, 0), 2); 
            putText(img, cv::format("OBJ: %ld, FEAT: %ld", object_id, feature_id), 
                    Point(selection.x, 
                          selection.y + selection.height/2 - 5), 
                    0, 0.25, cv::Scalar(200, 200, 200), 1);
            cv::imshow("LEFT first frame", img);

	    // Found frame: removing the older frames
            // cbuffer.erase(cbuffer.begin() + j + 1, cbuffer.end());
            std::cerr << "Found image" << utime << "=" << cbuffer[j].first << "~" << cbuffer[j+1].first << std::endl;
            break;
        }
    }
    //pthread_mutex_unlock(&buffer_mutex);

    return;
}

static void on_segmentation_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const perception_image_roi_t *msg, 
                            void *user_data ) {
    
    std::cerr << "SEGMENTATION msg: " << msg->utime << " " << msg->roi.x << " " << msg->roi.y << " " << 
        msg->roi.width << " " << msg->roi.height << std::endl;

    cv::Rect roi(msg->roi.x, msg->roi.y, msg->roi.width, msg->roi.height);
    select_object(msg->utime, msg->feature_id, msg->object_id, roi);
    return;
}

void pipe_to_buffer(int64_t utime, image_pair& img) { 
    //pthread_mutex_lock(&buffer_mutex);
    // Push to circular buffer
    cbuffer.push_front(std::make_pair(utime, img));

    // Circular buffer implementation to allow for delay in user-segmentation
    if (cbuffer.size()) { 
        const std::pair<int64_t, image_pair>& latest = cbuffer.front();

        // Keep only BUFFER_TIME_SIZE seconds of buffer
        int jtime;
        for (jtime=0; jtime<cbuffer.size(); jtime++) { 
            if (latest.first - cbuffer[jtime].first > BUFFER_TIME_SIZE * 1e6)
                break;
        }
        if (jtime < cbuffer.size()) { 
            // std::cerr << "Clearing up " << cbuffer.size() - jtime << std::endl;
            cbuffer.erase(cbuffer.begin() + jtime, cbuffer.end());
        }
    }
    //pthread_mutex_unlock(&buffer_mutex);
    return;
}


bool viz_tracker() { 
    //pthread_mutex_lock(&buffer_mutex);

    // Viz
    vs_point3d_list_collection_t viz_affordance_msg;
    viz_affordance_msg.id = 1234; 
    viz_affordance_msg.name = "AFFORDANCE ESTIMATES"; 
    viz_affordance_msg.type = VS_POINT3D_LIST_COLLECTION_T_LINE_LOOP; 
    viz_affordance_msg.reset = true; 

    cv::Scalar yellow(255,255,0);
    cv::Scalar blue(0,0,255);
    cv::Scalar black(0,0,0);
    cv::Scalar white(255,255,255);

    image_pair stereo_img = cbuffer.front().second;
    image_pair stereo_img_viz = cbuffer.front().second;
    
    if (stereo_img.first.channels() == 1)
        cvtColor(stereo_img.first, stereo_img_viz.first, CV_GRAY2BGR); 
    else
        stereo_img_viz.first = stereo_img.first; // .clone(); 

    // TLD Tracker
    cv::Rect null_rect = cv::Rect(-1,-1,1,1);

    std::vector<vs_point3d_list_t> objs;
    
    std::vector<cv::Scalar> colors(5); 
    opencv_utils::fillColors(colors); 
    for (TLDTrackerMapIt it = tld_tracker_map.begin(); it != tld_tracker_map.end(); it++) { 
        TLDInfo* tinfo = it->second; 
        int conf = (tinfo->tld->currConf >= .5) ? 1 : 0;
        cv::Mat& img = (it->first.second == 0) ? stereo_img_viz.first : stereo_img_viz.second; 
        
        if(tinfo->tld->currBB != NULL) {
            cv::Rect currBB = *(tinfo->tld->currBB);

            cv::Scalar rectangleColor = (conf) ? blue : yellow;
            cv::Scalar id_color = colors[tinfo->object_id%colors.size()];
            rectangle(img, cv::Point(currBB.tl()), cv::Point(currBB.br()), 
                      id_color, 2, 2, 0);   
            cv::Point bl = currBB.tl(); bl.y += currBB.height-5;
            putText(img, cv::format("OBJ: %ld, FEAT: %ld", tinfo->object_id, tinfo->feature_id), 
                    bl, 0, 0.25, cv::Scalar(200, 200, 200), 1);

            // std::cerr << "downscaled currBB: " << currBB << std::endl;
            int nrw = currBB.width*(1.f/vSCALE), nrh = currBB.height*(1.f/vSCALE);
            currBB.x = (currBB.x + currBB.width/2)*(1.f/vSCALE)-nrw/2, 
                currBB.y = (currBB.y + currBB.height/2)*(1.f/vSCALE)-nrh/2;
            currBB.width = nrw, currBB.height = nrh;
            // std::cerr << "upscaled currBB: " << currBB << std::endl;
                
            if (tinfo->tld->medianFlowTracker->trackerBB == NULL) { 
                tinfo->tld->trackerEnabled = true; 
            }

            if (it->first.second == 0) // left
                stereo_roi_map[it->first.first].left = currBB; 
            else 
                stereo_roi_map[it->first.first].right = currBB; 


            float stereo_depth = stereo_bm_depth(currBB, tinfo->last_processed);
            // std::cerr << "DEPTH: " << tinfo->object_id << " " << stereo_depth << std::endl;

            // std::cerr << "C: " << currBB.tl() << " " << tinfo->tld->currBB->br() << std::endl;
            cv::Point2f c((currBB.tl().x + currBB.br().x) * 0.5, 
                          (currBB.tl().y + currBB.br().y) * 0.5);

            cv::Point3f pt3d = camera_params["CAMERALEFT"].reprojectTo3D(c, stereo_depth);
            float h = currBB.br().y - currBB.tl().y; 
            float w = currBB.br().x - currBB.tl().x; 
            cv::Point3f pttl = camera_params["CAMERALEFT"].reprojectTo3D(currBB.tl(), stereo_depth);
            cv::Point3f pttr = camera_params["CAMERALEFT"].reprojectTo3D(cv::Point2f(currBB.br().x, 
                                                                                     currBB.tl().y), stereo_depth);
            cv::Point3f ptbr = camera_params["CAMERALEFT"].reprojectTo3D(currBB.br(), stereo_depth);
            cv::Point3f ptbl = camera_params["CAMERALEFT"].reprojectTo3D(cv::Point2f(currBB.tl().x, 
                                                                                     currBB.br().y), stereo_depth);

            // std::cerr << "pts: " << pttl << " " << pttr << " " << ptbr << " " << ptbl << std::endl;

            vs_point3d_list_t obj; 
            obj.id = bot_timestamp_now(); 
            obj.collection = 10000; 
            obj.element_id = 1; // obj.id; 

            int npts = 18;
            obj.npoints = npts; 
            obj.points = new vs_point3d_t[npts];
            float d = 0.1;
            vs_point3d_t* pts = obj.points; 
            // front
            pts[0].x = pttl.x, pts[0].y = pttl.y, pts[0].z = pttl.z - d/2; 
            pts[1].x = pttr.x, pts[1].y = pttr.y, pts[1].z = pttr.z - d/2; 
            pts[2].x = ptbr.x, pts[2].y = ptbr.y, pts[2].z = ptbr.z - d/2; 
            pts[3].x = ptbl.x, pts[3].y = ptbl.y, pts[3].z = ptbl.z - d/2; 

            // top
            pts[4].x = pttl.x, pts[4].y = pttl.y, pts[4].z = pttl.z - d/2; 
            pts[5].x = pttl.x, pts[5].y = pttl.y, pts[5].z = pttl.z + d/2; 
            pts[6].x = pttr.x, pts[6].y = pttr.y, pts[6].z = pttr.z + d/2; 
            pts[7].x = pttr.x, pts[7].y = pttr.y, pts[7].z = pttr.z - d/2; 

            // bottom
            pts[8].x = ptbr.x, pts[8].y = ptbr.y, pts[8].z = ptbr.z - d/2; 
            pts[9].x = ptbr.x, pts[9].y = ptbr.y, pts[9].z = ptbr.z + d/2; 
            pts[10].x = ptbl.x, pts[10].y = ptbl.y, pts[10].z = ptbl.z + d/2; 
            pts[11].x = ptbl.x, pts[11].y = ptbl.y, pts[11].z = ptbl.z - d/2; 

            pts[12].x = ptbl.x, pts[12].y = ptbl.y, pts[12].z = ptbl.z + d/2; 
            pts[13].x = pttl.x, pts[13].y = pttl.y, pts[13].z = pttl.z + d/2; 
            pts[14].x = pttr.x, pts[14].y = pttr.y, pts[14].z = pttr.z + d/2; 
            pts[15].x = ptbr.x, pts[15].y = ptbr.y, pts[15].z = ptbr.z + d/2; 
            pts[16].x = ptbl.x, pts[16].y = ptbl.y, pts[16].z = ptbl.z + d/2; 

            pts[17].x = pttl.x, pts[17].y = pttl.y, pts[17].z = pttl.z + d/2; 

            obj.ncolors = npts; 
            obj.colors = new vs_color_t[npts];
            for (int j=0; j<npts; j++) 
                obj.colors[j].r = id_color[2] / 255.f, obj.colors[j].g = id_color[1] / 255.f, obj.colors[j].b = id_color[0] / 255.f;
            
            obj.nnormals = 0; 
            obj.npointids = 0; 
            
            objs.push_back(obj); 
        }
    }

    viz_affordance_msg.nlists = objs.size(); 
    viz_affordance_msg.point_lists = &objs[0];    
    vs_point3d_list_collection_t_publish(state->lcm, "POINTS_COLLECTION", &viz_affordance_msg); 
    for (int j=0; j<objs.size(); j++) { 
        delete [] objs[j].points;
        delete [] objs[j].colors; 
    }

    cv::imshow("MULTISENSE LEFT", stereo_img_viz.first);
    //pthread_mutex_unlock(&buffer_mutex);
    return true;
}

static void on_depth_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {
    if (!msg->width || !msg->height) return;
    std::cerr << "on_depth_frame" << std::endl;

    std::cerr << " DEPTH: " << msg->height << " " << msg->width << std::endl;

    if (msg->pixelformat != BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32) return;
    
    cv::Mat_<float> depth = cv::Mat_<float>::zeros(msg->height, msg->width); 
    memcpy(depth.data, msg->data, msg->height * msg->width * sizeof(float));
    
    if (depth.empty()) return;
    cv::imshow("depth", depth);

    if (depth_buffer.size()) depth_buffer.pop_front();
    depth_buffer.push_front(depth);     
    return;
}

static void on_affordance_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {

    std::cerr << "on_affordance_frame" << std::endl;
    if (!msg->width || !msg->height) return;

    state_t* state = (state_t*) user_data; 

    // Data lies 
    uint8_t* data = (uint8_t*) state->imgutils->unzipImage(msg);
    state->aff_img = cv::Mat( msg->height , msg->width , CV_8UC1, data);
    state->aff_utime = msg->utime; 
    return;
}

void affordance_capture(state_t* state) { 
    if (state->aff_img.empty()) { 
        std::cerr << esc_red << " *********** AFFORDANCE EMPTY  *********** " << esc_def << std::endl;
        return;
    }
    if (vDEBUG)
        std::cerr << "AFFORDANCE CAPTURE: " << std::endl;
    double tic = bot_timestamp_now(); 
    unsigned char* aff_imgp = state->aff_img.data; 

    std::set<int32_t> unique_ids; 
    for (int y=0; y<state->aff_img.rows; y++, 
             aff_imgp += state->aff_img.step) { 
        for (int x=0; x<state->aff_img.cols; x++) { 
            int id = int(aff_imgp[x]);
            if (id == 255 || id == 0) continue;
            if (unique_ids.find(id) == unique_ids.end())
                unique_ids.insert(id); 
        }
    }

    int SEARCH_SCALE = 4; 
    cv::Mat aff_img_resized; 
    cv::resize(state->aff_img, aff_img_resized, cv::Size(), vSCALE, vSCALE, cv::INTER_LINEAR);
    if (vDEBUG) 
        std::cerr << "AFFORDANCE_IMAGE *************> " << aff_img_resized.cols << "x" << aff_img_resized.rows << std::endl;

    for (std::set<int32_t>::iterator it = unique_ids.begin(); it != unique_ids.end(); it++) { 
        aff_imgp = state->aff_img.data; 

        int minx = std::numeric_limits<int>::max(), miny = minx; 
        int maxx = std::numeric_limits<int>::min(), maxy = maxx;

        for (int y=0; y<state->aff_img.rows; y+=SEARCH_SCALE, 
                 aff_imgp += SEARCH_SCALE*state->aff_img.step) { 
            for (int x=0; x<state->aff_img.cols; x+=SEARCH_SCALE) { 
                int id = int(aff_imgp[x]);
                if (id != *it) continue;
                if (x < minx) minx = x; 
                if (y < miny) miny = y; 
                if (x > maxx) maxx = x; 
                if (y > maxy) maxy = y; 
            }
        }
        int rx = minx - 5, ry = miny - 5; 
        int rw = maxx-minx + 10, rh = maxy-miny + 10;

        
        if ((rx < 0) || (ry < 0) 
            || (rw >= state->aff_img.cols) || (rh >= state->aff_img.rows)
            || (rw <= vMIN_DIM) || (rh <= vMIN_DIM)
            || (rx + rw > state->aff_img.cols) || (ry + rh > state->aff_img.rows) ) { 
            std::cerr << "FAILED to initialize ID: " << *it << " dims (not valid)" << std::endl;
            continue;
        } else { 
            if (vDEBUG)
                std::cerr << "INITIALIZING ID: " << *it << std::endl;
        }
        cv::Rect rect(rx,ry,rw,rh);
        cv::rectangle(state->aff_img, rect, CV_RGB(255, 255, 255), 1); 
        // std::cerr << "aff upscaled Rect: " << rect << std::endl;

        int nrw = rw * vSCALE, nrh = rh * vSCALE;
        cv::Rect up_rect((rx + rw/2)*vSCALE - nrw/2,(ry + rh/2)*vSCALE - nrh/2,nrw,nrh);
        cv::rectangle(aff_img_resized, up_rect, CV_RGB(255, 255, 255), 1); 

        // std::cerr << "aff downscaled Rect: " <<  up_rect << std::endl;
        affordance_roi_map[*it] = TimedRect(state->aff_utime, up_rect.tl().x, up_rect.tl().y, up_rect.width, up_rect.height); 

    }
    // cv::imshow("img", img); 
    // cv::imshow("Mask", mask); 
    // color_tracker_map[1]->initialize(img, mask);

    if (vDEBUG && state->aff_img.data) { 
        std::cerr << "affordance msg: " << state->aff_utime << std::endl;
        cv::imshow("Affordance", state->aff_img); 
        cv::imshow("Affordance Resized", aff_img_resized); 
    }
    printf("===> AFFORDANCE_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
    return;
}

void process_latest_frame(TLDInfo* tinfo, std::deque<std::pair<int64_t, cv::Mat> >& lbuffer) {
    std::pair<int64_t, cv::Mat>& bufpair = lbuffer.back();

    if (lbuffer.size() == 1) 
        std::cerr << esc_green << "LIVE TRACKING: " << tinfo->tld->medianFlowTracker->trackerBB << esc_def << std::endl;
    else 
        std::cerr << esc_yellow << "BUFFERED TRACKING " << esc_def << std::endl;
    
    // std::cerr << esc_green 
    //           << "utime: " << lbuffer.back().first 
    //           << " processImage: " << bufpair.second.rows << "x" << bufpair.second.cols << esc_def << std::endl;


    // Track each of the remaining buffers before going live
    tinfo->tld->processImage(bufpair.second);
    tinfo->init_utime = lbuffer.back().first; 
    lbuffer.pop_back();
    return;
}

void run_tracker() { 

    double tic = bot_timestamp_now();
    for (TLDTrackerMapIt it = tld_tracker_map.begin(); it != tld_tracker_map.end(); it++) { 
        TLDInfo* tinfo = it->second; 
        if (!(tinfo->inited)) continue;

        // std::cerr << ((it->first.second == 0) ? "LEFT" : "DEPTH") << " CAMERA" << std::endl;
        std::deque<std::pair<int64_t, cv::Mat> > lbuffer; 

        // std::cerr << "INIT UTIME: " << tinfo->init_utime * 1e-6 << std::endl;
        for (int j=0; j<cbuffer.size()-1; j++) { 
            if (cbuffer[j].first >= tinfo->init_utime && cbuffer[j+1].first < tinfo->init_utime) 
                break;
            // std::cerr << "bw: "<< cbuffer[j].first * 1e-6 << " " << cbuffer[j+1].first * 1e-6 << std::endl;
            lbuffer.push_back(std::make_pair(cbuffer[j].first, 
                                             (it->first.second == 0) ? cbuffer[j].second.first : cbuffer[j].second.second)); 
        }
        std::cerr << esc_red << "PENDING: " << lbuffer.size() << esc_def << std::endl;
        // Track each FEAT_ID/OBJECT_ID independently
        if (lbuffer.size() > 1) { 
            // Once the buffer is clean
            int j = 0, count = 2;
            while (j < count && lbuffer.size()) { 

                tinfo->last_processed = 0; // don't count as processed yet 
                process_latest_frame(tinfo, lbuffer); 

                if(tinfo->tld->currBB != NULL)
                    std::cerr << esc_yellow << "OBJ_ID: " << tinfo->object_id << " FEAT_ID: " << tinfo->feature_id 
                              << " | Pending frames " << lbuffer.size() << " | " << *(tinfo->tld->currBB) << esc_def << std::endl;
                else 
                    std::cerr << esc_yellow << "OBJ_ID: " << tinfo->object_id << " FEAT_ID: " << tinfo->feature_id 
                              << " | Pending frames " << lbuffer.size() << " | NOT FOUND" << esc_def << std::endl;
            }
        } else if (lbuffer.size() == 1) { 
            // std::cerr << "Live tracking " << msg->utime << std::endl;
            // Live Tracking 
            if (tinfo->tld->medianFlowTracker->trackerBB != NULL) { 
                std::cerr << "MEDIAN FLOW TRACKER: " << *(tinfo->tld->medianFlowTracker->trackerBB) << std::endl;
            }
            
            if (lbuffer.back().first - tinfo->last_processed >= (1.f / vDETECT_FPS) * 1e6) { 
                std::cerr << esc_red << " --------- DETECT---------"  
                          << lbuffer.back().first - tinfo->last_processed << esc_def << std::endl;

                tinfo->last_processed = lbuffer.back().first;
                process_latest_frame(tinfo, lbuffer); 
            } else { 
                tinfo->init_utime = lbuffer.back().first; 
                lbuffer.pop_back();
            }

            // if (!tinfo->tld->trackerEnabled) { 
            //     tinfo->init_utime = lbuffer.back().first; 
            //     lbuffer.pop_back();
            // }
            

                
        } else 
            std::cerr << "Empty buffer: " << lbuffer.size() << std::endl;

        std::cerr << "DONE TRACKING OBJ_ID: " << tinfo->object_id 
                  << " | FEAT_ID: " << tinfo->feature_id 
                  << " | BUFFER: " << lbuffer.size() << std::endl;

        printf("=====> TLD_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
    }
    // printf("=====> IMAGE_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
    return;
}

void viz_test(cv::Mat& img, std::vector<cv::KeyPoint>& kpts) { 
    for (int j=0; j<kpts.size(); j++) {
        float angle = kpts[j].angle;
        if ( angle < 0) angle = 0;
        // cv::RotatedRect box(kpts[j].pt, cv::Size(kpts[j].size,10), angle); 
        circle(img, kpts[j].pt, 2, CV_RGB(240,240,240), 1);
        // cv::ellipse( img, box, Scalar(196,255,255), 1 );
        // std::cerr << " pt: " << kpts[j].pt << " sz: " << kpts[j].size << " a: " << kpts[j].angle 
        //           << " r: " << kpts[j].response << " id: " << kpts[j].class_id << std::endl;
    }
    imshow("test", img);
}

void process_left_and_depth_images(int64_t utime, const cv::Mat& img, const cv::Mat& depth) { 
    // Ensure that the buffer always has a few seconds worth 
    // of frames (depending on network latency) 
    double tic = bot_timestamp_now(); 

    image_pair left_depth_pair = std::make_pair(img, depth);
    pipe_to_buffer(utime, left_depth_pair); 
    printf("=====> PIPE_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
}


static void on_multisense_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                                 const multisense_images_t *msg, 
                            void *user_data ) {
    if (!msg->n_images) return;
    state_t* state = (state_t*) user_data; 

    // if (cbuffer.size() && ((msg->utime - cbuffer.front().first) <= (1.f / vTRACKER_FPS) * 1e6))
    //     return;

    std::cerr << esc_cyan << "===========> on_multisense_frame <=========== " << msg->utime * 1e-6 << esc_def << std::endl;
    double tic = bot_timestamp_now(); 
    // Copy over DEPTH and LEFT images
    for (int j=0; j<msg->n_images; j++) { 
        // Image Type
        if (msg->image_types[j] == MULTISENSE_IMAGES_T_LEFT) { 
            // Create images
            if (state->img.empty() || state->gray.empty()) { 
                state->gray.create(msg->images[j].height, msg->images[j].width, CV_8UC1); 
                state->img.create(msg->images[j].height, msg->images[j].width, CV_8UC3); 
            }               
            // Copy images
            if (msg->images[j].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) { 
                decode_image(&(msg->images[j]), state->gray); 
                state->img = state->gray.clone(); 
            } else if (msg->images[j].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {  
                decode_image(&(msg->images[j]), state->img); 
                state->gray = state->img.clone();                     
            }
            
        } // Depth Image
        else if (msg->image_types[j] == MULTISENSE_IMAGES_T_DISPARITY) {
            assert(msg->images[j].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_FLOAT_GRAY32);
            // std::cerr << "CREATING depth: " 
            // << msg->images[j].height << "x" << msg->images[j].width << "=" << msg->images[j].size << std::endl;
            state->depth16 = cv::Mat(msg->images[j].height, msg->images[j].width,CV_16UC1, msg->images[j].data);
            state->depth16.convertTo(state->depth, state->depth.type(), 1.f / 16.f); 
            if (vDEBUG)
                state->depth16.convertTo(state->depth8, state->depth8.type(), 1/16.f); 

        }
    }
    printf("=====> DECODE_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);

    if (state->depth.empty() || state->img.empty()) { 
        std::cerr << "Either DEPTH or IMAGE is empty()" << std::endl;
        return;
    }

    int method = cv::INTER_LINEAR; //cv::INTER_AREA : cv::INTER_CUBIC;
    cv::resize(state->img, state->img, cv::Size(), vSCALE, vSCALE, method);
    // cv::resize(state->depth, state->depth, cv::Size(), vSCALE, vSCALE, method);
    std::cerr << "=========== > RESIZED: " << state->img.cols << "x" << state->img.rows << std::endl;

    // Make sure camera frame is published for viz
    publish_camera_frame(msg->utime);

    // Capture all the affordances ROI 
    affordance_capture(state);

    // Main process of LEFT AND DISPARITY
    tic = bot_timestamp_now(); 
    process_left_and_depth_images(msg->utime, state->img, state->depth); 
    printf("=====> PROCESS LEFT AND DEPTH: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);

    // Run Tracker
    tic = bot_timestamp_now(); 
    run_tracker(); 
    printf("=====> RUN TRACKER: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
    
    // Viz Tracker
    tic = bot_timestamp_now(); 
    if (!viz_tracker()) { 
        if (!(state->img.empty()))
            cv::imshow("MULTISENSE LEFT", state->img); 
    }
    printf("=====> VIZ TRACKER: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);    

    if (vDEBUG && !state->depth8.empty())
        cv::imshow("MULTISENSE DEPTH", state->depth8); 
    return; 
}


double in_tic = 0; 
static void on_image_frame (const lcm_recv_buf_t *rbuf, const char *channel,
                            const bot_core_image_t *msg, 
                            void *user_data ) {
    if (!msg->width || !msg->height)
        return;
        
    state_t* state = (state_t*) user_data; 

    double now = bot_timestamp_now(); 
    std::cerr << "on_image_frame FPS: " << 1 / ((now - in_tic) * 1e-6)  << std::endl;
    in_tic = now;

    double tic = bot_timestamp_now(); 

    // cv::Mat& img = state->img; 
    // cv::Mat& gray = state->gray; 
    cv::Mat img, gray; 

    if (img.empty() || gray.empty() || img.rows != msg->height || img.cols != msg->width) { 
        if (msg->pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY)
            gray.create(msg->height, msg->width, CV_8UC1);
        else 
            img.create(msg->height, msg->width, CV_8UC3);
    }

    if (img.channels() == 1)
        decode_image(msg, gray); 
    else 
        decode_image(msg, img); 

    if (img.channels() == 3) { 
        cvtColor(img, img, CV_RGB2BGR);
        cvtColor(img, gray, CV_RGB2GRAY);
    } else { 
        cvtColor(gray, img, CV_GRAY2BGR);
    }
    printf("DECODE_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);

    cv::Mat left_stereo, right_stereo; 
    const int STEREO_WIDTH=img.cols, STEREO_HEIGHT=img.rows/2; 
    if (vSTEREO_MODE) { 
        left_stereo = Mat(img, Rect(0,0,STEREO_WIDTH,STEREO_HEIGHT)).clone(); 
        right_stereo = Mat(img, Rect(0,STEREO_HEIGHT,STEREO_WIDTH,STEREO_HEIGHT)).clone(); 
    } else { 
        left_stereo = img.clone();
        right_stereo = cv::Mat();
    }

    // Ensure that the buffer always has a few seconds worth 
    // of frames (depending on network latency) 
    image_pair stereo_img = std::make_pair(left_stereo, right_stereo);
    pipe_to_buffer(msg->utime, stereo_img); 
        
    // printf("PIPE_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);

    // // // // klt_tracker
    // // // if (klt_tracker) 
    // // //     klt_tracker->update(msg->utime, img); 
    // // color_tracker_map[1]->update(left_stereo); 
    

    // if (viz_tracker())
    //     return; 

    // printf("VIZ_FRAME: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);

    // // cv::imshow(WINDOW_NAME, img);

    // if (left_stereo.empty() || right_stereo.empty())
    //     return;

    // cv::imshow("left", left_stereo);
    // cv::imshow("right", right_stereo);
    return;
}

void set_camera_params(std::string cam) { 
    std::string key_prefix_str = "cameras."+cam+".intrinsic_cal";
    camera_params[cam].width = bot_param_get_int_or_fail(state->param, (key_prefix_str+".width").c_str());
    camera_params[cam].height = bot_param_get_int_or_fail(state->param,(key_prefix_str+".height").c_str());
    camera_params[cam].setIntrinsics(bot_param_get_double_or_fail(state->param, (key_prefix_str+".fx").c_str()), 
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".fy").c_str()), 
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".cx").c_str()), 
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".cy").c_str()), 
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".k1").c_str()), 
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".k2").c_str()),
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".k3").c_str()),
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".p1").c_str()),
                                     bot_param_get_double_or_fail(state->param, (key_prefix_str+".p2").c_str())); 
    return;
}


void publish_camera_frame(int64_t utime) { 
    BotTrans sensor_frame;
    bot_frames_get_trans_with_utime (state->frames, "CAMERA", "local", utime, &sensor_frame);
    double rpy[3]; bot_quat_to_roll_pitch_yaw(sensor_frame.rot_quat, rpy);
        
    vs_obj_collection_t objs_msg; 
    objs_msg.id = 10000; 
    objs_msg.name = "CAMERA_POSE"; 
    objs_msg.type = VS_OBJ_COLLECTION_T_AXIS3D; 
    objs_msg.reset = true; 
    vs_obj_t poses[1]; 
    poses[0].id = 1; 
    poses[0].x = sensor_frame.trans_vec[0], 
        poses[0].y = sensor_frame.trans_vec[1], poses[0].z = sensor_frame.trans_vec[2]; 
    poses[0].roll = rpy[0], poses[0].pitch = rpy[1], poses[0].yaw = rpy[2]; 
    objs_msg.nobjs = 1; 
    objs_msg.objs = &poses[0];
    vs_obj_collection_t_publish(state->lcm, "OBJ_COLLECTION", &objs_msg);
    return;
}

void write_tldcascade_to_file() { 
    for (TLDTrackerMapIt it = tld_tracker_map.begin(); it != tld_tracker_map.end(); it++) { 
        TLDInfo* tinfo = it->second; 
        std::stringstream ss; 
        ss << "model-id-" << it->first.first;
        std::cerr << esc_green << "WRITING model: " << ss.str() << esc_def << std::endl;
        tinfo->tld->writeToFile(ss.str().c_str());
    }
    return;
}

void read_tldcascade_from_file() { 
    if (!cbuffer.size()) return;

    double tic = bot_timestamp_now();
    affordance_capture(state); 
    printf("===> AFFORDANCE_CAPTURE: %f ms\n", (bot_timestamp_now() - tic) * 1e-3);
    std::cerr << "AFFORDANCE CAPTURE: " << std::endl;
    for (std::map<int32_t, TimedRect>::iterator it = affordance_roi_map.begin(); 
         it != affordance_roi_map.end(); it++) {

        int32_t object_id = it->first;
        int32_t feature_id = 0; 
        std::stringstream ss; 
        ss << "model-id-" << object_id;
        std::cerr << esc_yellow << "READING model: " << ss.str() << esc_def << std::endl;
    
        int64_t utime = cbuffer.front().first;
        std::pair<int32_t, int32_t> sel_hash = std::make_pair(object_id, 0); 
        if (tld_tracker_map.find(sel_hash) != tld_tracker_map.end()) { 
            std::cerr << "ReInitializing OBJ: " << object_id 
                      << "FEAT: " << feature_id << std::endl;
            tld_tracker_map.erase(sel_hash); 
        } else { 
            std::cerr << "Initializing OBJ: " << object_id 
                      << "FEAT: " << feature_id << std::endl;
        }

        TLDInfo* tinfo = new TLDInfo(utime, sel_hash.first, sel_hash.second); // camera left
        if (tinfo->tld->readFromFile(ss.str().c_str())) { 
            tinfo->tld->learningEnabled = false;
        } else { 
            delete tinfo;
            std::cerr << esc_red << "FAILED READING model: " << ss.str() << esc_def << std::endl;
            continue;
        }
        std::cerr << esc_green << "ADDING model: " << ss.str() << esc_def << std::endl;
        tld_tracker_map.insert(std::make_pair(sel_hash, tinfo)); 
    }

}

int main(int argc, char** argv)
{
    // g_thread_init(NULL);
    setlinebuf (stdout);
    state = new state_t();

    // Param server, botframes
    state->lcm =  bot_lcm_get_global(NULL);
    // state->mainloop = g_main_loop_new( NULL, FALSE );  
    state->param = bot_param_new_from_server(state->lcm, 1);
    state->frames = bot_frames_get_global (state->lcm, state->param);
    state->imgutils = new image_io_utils( state->lcm, 1024, 544);

    // Start LCM thread
    printf("Starting LCM thread\n");

    ConciseArgs opt(argc, (char**)argv);
    opt.add(vSTEREO_MODE, "s", "stereo-mode", "Stereo Mode");
    opt.add(vAFFORDANCE_ID, "i", "affordance-id", "AFFORDANCE ID"); 
    opt.add(vAFFORDANCE_CHANNEL, "ac", "affordance-channel", "AFFORDANCE CHANNEL"); 
    opt.add(vAFFORDANCE_MODE, "m", "affordance-mode","0=rgbinJPEGREDUCEDOUT 1=zipinGRAYOUT");
    opt.add(vSCALE, "c", "scale","scale");\
    opt.add(vDETECT_FPS, "f", "dfps","DETECTION FPS");
    opt.add(vTRACKER_FPS, "t", "tfps","TRACKER FPS");
    opt.add(vDEBUG, "d", "debug", "Debug mode");
    opt.parse();
  
    std::cerr << esc_yellow ;
    std::cerr << "===========  Stereo Object Tracker ============" << std::endl;
    std::cerr << "=> STEREO_MODE : " << vSTEREO_MODE << std::endl;
    std::cerr << "=> SCALE : " << vSCALE << std::endl;
    std::cerr << "=> TRACKER FPS : " << vTRACKER_FPS << std::endl;
    std::cerr << "=> DETECT FPS : " << vDETECT_FPS << std::endl;
    std::cerr << "=> DEBUG : " << vDEBUG << std::endl;
    std::cerr << "=> AFFORDANCE_CHANNEL : " << vAFFORDANCE_CHANNEL << std::endl;
    std::cerr << "=> TRACKING AFFORDANCE_ID " << vAFFORDANCE_ID << " on " << vAFFORDANCE_CHANNEL << std::endl;
    std::cerr << "===============================================" << std::endl;
    std::cerr << esc_def ;
   
    // Debug
    // klt_tracker = new MRPTTracker(); 
    
    // Set Camera Params
    set_camera_params(std::string("CAMERALEFT"));
    set_camera_params(std::string("CAMERARIGHT"));

    // Determine [R t] from CAMERARIGHT in CAMERALEFT's frame of ref. 
    BotTrans right_frame;
    bot_frames_get_trans (state->frames, "CAMERA", "CAMERARIGHT", &right_frame);
    double stereoR[9]; bot_quat_to_matrix(right_frame.rot_quat, stereoR);
    camera_params[std::string("CAMERARIGHT")].setExtrinsics(&stereoR[0], &right_frame.trans_vec[0]); 

    // bot_core_image_t_subscribe(state->lcm, "CAMERALEFT", on_image_frame, (void*)state);
    multisense_images_t_subscribe(state->lcm, "MULTISENSE_LD", on_multisense_frame, (void*)state);
    bot_core_image_t_subscribe(state->lcm, vAFFORDANCE_CHANNEL.c_str(), on_affordance_frame, (void*)state);
    //perception_image_roi_t_subscribe(state->lcm, "TLD_OBJECT_ROI", on_segmentation_frame, (void*)state);

    // Install signal handler to free data.
    signal(SIGINT, INThandler);

    fprintf(stderr, "Starting Main Loop\n");
    

    // HistTracker* tr = new HistTracker(); 
    // color_tracker_map[1] = tr; 


    // LINEMODInfo* linfo = new LINEMODInfo(0,65,0); 
    // linemod_tracker_map[1] = linfo; 

   
    while(1) { 
        unsigned char c = cv::waitKey(1) & 0xff;
	lcm_handle(state->lcm);
        if (c == 'q') { 
            break;  
        } else if ( c == 's' ) {
            affordance_roi_map.clear();
            affordance_capture(state); 
            std::cerr << "AFFORDANCE CAPTURE: " << std::endl;
            for (std::map<int32_t, TimedRect>::iterator it = affordance_roi_map.begin(); 
                 it != affordance_roi_map.end(); it++) {
                cv::Rect r(it->second.x, it->second.y, 
                           it->second.width, it->second.height) ;
                // std::cerr < "INITIALIZING: " << it->first << " @ " << r << " utime: " 
                //           << it->second.utime << std::endl;
                select_object(it->second.utime, it->first, 0, r); 
            }

            // } else if (c == 'l') { 
        //     tldtracker->learningEnabled = !tldtracker->learningEnabled;
        //     printf("LearningEnabled: %d\n", tldtracker->learningEnabled);
        // } else if (c == 'a') {
        //     tldtracker->alternating = !tldtracker->alternating;
        //     printf("alternating: %d\n", tldtracker->alternating);
        } else if (c == 'r') { 
            read_tldcascade_from_file();
        } else if (c == 'w') { 
            write_tldcascade_to_file(); 
        }
    }

    lcm_destroy(state->lcm);
    write_tldcascade_to_file(); 
    
    return 0;
}

