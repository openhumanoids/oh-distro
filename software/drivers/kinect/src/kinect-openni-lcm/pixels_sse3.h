#ifndef __PIXELS_SSE3_H__
#define __PIXELS_SSE3_H__

#include <stdint.h>
#include "pixels.h"

int
cam_pixel_bayer_interpolate_to_8u_bgra_sse3 (uint8_t ** src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format);
int
cam_pixel_bayer_interpolate_to_8u_gray_sse3 (uint8_t * src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format);

#endif
