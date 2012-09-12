#include <iostream>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_frames/bot_frames.h>

#include "pointcloud_tools.hpp"

#include "visualization/collections.hpp"

#include <zlib.h>

#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;


pointcloud_tools::pointcloud_tools (lcm_t* publish_lcm):
    publish_lcm_(publish_lcm){

}


