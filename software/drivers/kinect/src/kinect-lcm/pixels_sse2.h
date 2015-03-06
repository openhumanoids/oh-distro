#ifndef __PIXELS_SSE2_H__
#define __PIXELS_SSE2_H__

#include <stdint.h>
#include "pixels.h"

int
cam_pixel_split_bayer_planes_8u_sse2 (uint8_t *dst[4], int dstride,
        const uint8_t * src, int sstride, int width, int height);
int
cam_pixel_bayer_interpolate_to_8u_bgra_sse2 (uint8_t ** src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format);
int
cam_pixel_bayer_interpolate_to_8u_gray_sse2 (uint8_t * src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format);

#endif
