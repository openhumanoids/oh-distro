#include <iostream>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "pointcloud_lcm.hpp"

#include "visualization/collections.hpp"

//#include "jpeg-utils.h"
//#include "jpeg-utils-ijg.c"
//#include "jpeg-utils-ijg.h"

#include <zlib.h>

#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;


pointcloud_lcm::pointcloud_lcm (lcm_t* publish_lcm):
    publish_lcm_(publish_lcm){

}



