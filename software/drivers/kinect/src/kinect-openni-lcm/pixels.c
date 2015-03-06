#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "pixels.h"
#include "cpuid.h"
#include "pixels_sse2.h"
#include "pixels_sse3.h"

// HAVE_INTEL is defined in config.h by autotools
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef MIN
#define MIN(a,b) ( (a)<(b) ? (a) : (b) )
#endif
#ifndef MAX
#define MAX(a,b) ( (a)>(b) ? (a) : (b) )
#endif

#ifdef __APPLE__
#define MALLOC_ALIGNED(s) malloc(s)
#else
#include <malloc.h>
#define MALLOC_ALIGNED(s) memalign(16,s)
#endif

static int cpuid_detected = 0;
static int has_sse2;
static int has_sse3;

int
cam_pixel_format_bpp (CamPixelFormat p)
{
    switch (p) {
        case CAM_PIXEL_FORMAT_UYVY:
        case CAM_PIXEL_FORMAT_YUYV:
        case CAM_PIXEL_FORMAT_YUV411P:
            return 16;
        case CAM_PIXEL_FORMAT_IYU1:
            return 12;
        case CAM_PIXEL_FORMAT_IYU2:
        case CAM_PIXEL_FORMAT_RGB:
        case CAM_PIXEL_FORMAT_BGR:
            return 24;
        case CAM_PIXEL_FORMAT_YUV420:
//        case CAM_PIXEL_FORMAT_YV12:
        case CAM_PIXEL_FORMAT_I420:
            return 12;
        case CAM_PIXEL_FORMAT_NV12:
            return 10;
        case CAM_PIXEL_FORMAT_RGBA:
        case CAM_PIXEL_FORMAT_BGRA:
            return 32;
        case CAM_PIXEL_FORMAT_BE_RGB16:
        case CAM_PIXEL_FORMAT_LE_RGB16:
        case CAM_PIXEL_FORMAT_BE_SIGNED_RGB16:
            return 48;
        case CAM_PIXEL_FORMAT_GRAY:
        case CAM_PIXEL_FORMAT_BAYER_BGGR:
        case CAM_PIXEL_FORMAT_BAYER_GBRG:
        case CAM_PIXEL_FORMAT_BAYER_GRBG:
        case CAM_PIXEL_FORMAT_BAYER_RGGB:
            return 8;
        case CAM_PIXEL_FORMAT_BE_GRAY16:
        case CAM_PIXEL_FORMAT_LE_GRAY16:
        case CAM_PIXEL_FORMAT_BE_SIGNED_GRAY16:
        case CAM_PIXEL_FORMAT_BE_BAYER16_BGGR:
        case CAM_PIXEL_FORMAT_BE_BAYER16_GBRG:
        case CAM_PIXEL_FORMAT_BE_BAYER16_GRBG:
        case CAM_PIXEL_FORMAT_BE_BAYER16_RGGB:
        case CAM_PIXEL_FORMAT_LE_BAYER16_BGGR:
        case CAM_PIXEL_FORMAT_LE_BAYER16_GBRG:
        case CAM_PIXEL_FORMAT_LE_BAYER16_GRBG:
        case CAM_PIXEL_FORMAT_LE_BAYER16_RGGB:
            return 16;
        case CAM_PIXEL_FORMAT_MJPEG:
            return 12; /* worst-case estimate */
        case CAM_PIXEL_FORMAT_FLOAT_GRAY32:
            return 32;
//        case CAM_PIXEL_FORMAT_FLOAT_RGB32:
//            return 96;
        case CAM_PIXEL_FORMAT_INVALID:
        case CAM_PIXEL_FORMAT_ANY:
            return 0;
    }
    return 0;
}

int cam_pixel_format_stride_meaningful (CamPixelFormat p)
{
    switch (p) {
        case CAM_PIXEL_FORMAT_MJPEG:
        case CAM_PIXEL_FORMAT_INVALID:
            return 0;
        default:
            return 1;
    }
}

int
cam_pixel_convert_8u_gray_to_8u_RGB (uint8_t * dest, int dstride,
        int dwidth, int dheight, const uint8_t * src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[3*j] = drow[3*j+1] = drow[3*j+2] = srow[j];
        }
    }
    return 0;
}

int
cam_pixel_convert_8u_gray_to_64f_gray (double * dest, int dstride,
        int dwidth, int dheight, const uint8_t * src, int sstride)
{
    double s = 1 / 255.0;
    int i, j;
    for( i=0; i<dheight; i++ ) {
        double *drow = (double*)( (uint8_t*)dest + i * dstride );
        const uint8_t *srow = src + i * sstride;
        for( j=0; j<dwidth; j++ ) {
            drow[j] = srow[j] * s;
        }
    }
    return 0;
}

int cam_pixel_convert_8u_gray_to_32f_gray (float *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride)
{
    float s = 1 / 255.0;
    int i, j;
    for( i=0; i<dheight; i++ ) {
        float *drow = (float*)( (uint8_t*)dest + i * dstride );
        const uint8_t *srow = src + i * sstride;
        for( j=0; j<dwidth; j++ ) {
            drow[j] = srow[j] * s;
        }
    }
    return 0;
}

int cam_pixel_convert_32f_gray_to_8u_gray (uint8_t *dest, int dstride,
        int dwidth, int dheight, const float *src, int sstride)
{
    for(int i=0; i<dheight; i++ ) {
        uint8_t *drow = dest + i * dstride;
        const float *srow = (float*)( (uint8_t*)src + i * sstride );
        for(int j=0; j<dwidth; j++ ) {
            drow[j] = srow[j] * 255;
        }
    }
    return 0;
}

int
cam_pixel_convert_8u_gray_to_8u_RGBA (uint8_t * dest, int dstride,
        int dwidth, int dheight, const uint8_t * src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[4*j] = drow[4*j+1] = drow[4*j+2] = srow[j];
            drow[4*j+3] = 0xff;
        }
    }
    return 0;
}

int
cam_pixel_apply_lut_8u (uint8_t * dest, int dstride, int dwidth, int dheight,
        const uint8_t * src, int sstride, const uint8_t * lut)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[j] = lut[srow[j]];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
    int i, j;
    for (i=0; i<height; i++) {
        uint8_t *drow = dest + i * dstride;
        const uint8_t *srow = src + i * sstride;
        for (j=0; j<width; j++) {
            drow[j] = 0.2125 * srow[j*3+0] + 
                      0.7154 * srow[j*3+1] + 
                      0.0721 * srow[j*3+2];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_rgb_to_32f_gray (float *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
    float rw = 0.2125, gw = 0.7154, bw = 0.0721, s = 1 / 255.0;
    for (int i=0; i<height; i++) {
        const uint8_t *srow = src + i * sstride;
        float *drow = (float*) (((uint8_t*)dest) + i * dstride);
        for (int j=0; j<width; j++) {
            drow[j] = s * (rw*srow[3*j] + gw*srow[3*j+1] + bw*srow[3*j+2]);
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_rgb_to_8u_bgr(uint8_t *dest, int dstride, int dwidth, 
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[j*3 + 0] = srow[j*3 + 2];
            drow[j*3 + 1] = srow[j*3 + 1];
            drow[j*3 + 2] = srow[j*3 + 0];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_bgr_to_8u_rgb(uint8_t *dest, int dstride, int dwidth, 
        int dheight, const uint8_t *src, int sstride)
{
    return cam_pixel_convert_8u_rgb_to_8u_bgr(dest, dstride, dwidth, dheight, 
            src, sstride);
}

int 
cam_pixel_convert_8u_rgb_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[j*4 + 0] = srow[j*3 + 2];
            drow[j*4 + 1] = srow[j*3 + 1];
            drow[j*4 + 2] = srow[j*3 + 0];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_bgra_to_8u_bgr(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[j*3 + 0] = srow[j*4 + 0];
            drow[j*3 + 1] = srow[j*4 + 1];
            drow[j*3 + 2] = srow[j*4 + 2];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_bgra_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth; j++) {
            drow[j*3 + 0] = srow[j*4 + 2];
            drow[j*3 + 1] = srow[j*4 + 1];
            drow[j*3 + 2] = srow[j*4 + 0];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_yuv420p_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    const uint8_t *uplane = src + dheight*sstride;
    const uint8_t *vplane = uplane + dheight*sstride/4;

    for (int i=0; i<dheight/2; i++) {
        const uint8_t *yrow1 = src + i*2*sstride;
        const uint8_t *yrow2 = src + i*2*sstride + sstride;
        const uint8_t *urow = uplane + i*sstride/2;
        const uint8_t *vrow = vplane + i*sstride/2;
        uint8_t *rgb1 = dest + i*2*dstride;
        uint8_t *rgb2 = dest + i*2*dstride + dstride;
        for (int j=0; j<dwidth/2; j++) {
            int cb = ((urow[j]-128) * 454)>>8;
            int cr = ((vrow[j]-128) * 359)>>8;
            int cg = ((vrow[j]-128) * 183 + (urow[j]-128) * 88)>>8;
            int r, g, b, yp;

            yp = yrow1[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*6 + 0] = MAX(0, MIN(255, r));
            rgb1[j*6 + 1] = MAX(0, MIN(255, g));
            rgb1[j*6 + 2] = MAX(0, MIN(255, b));

            yp = yrow1[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*6 + 3] = MAX(0, MIN(255, r));
            rgb1[j*6 + 4] = MAX(0, MIN(255, g));
            rgb1[j*6 + 5] = MAX(0, MIN(255, b));

            yp = yrow2[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*6 + 0] = MAX(0, MIN(255, r));
            rgb2[j*6 + 1] = MAX(0, MIN(255, g));
            rgb2[j*6 + 2] = MAX(0, MIN(255, b));

            yp = yrow2[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*6 + 3] = MAX(0, MIN(255, r));
            rgb2[j*6 + 4] = MAX(0, MIN(255, g));
            rgb2[j*6 + 5] = MAX(0, MIN(255, b));

        }
    }
    return 0;
}

int
cam_pixel_convert_8u_yuv420p_to_8u_bgr(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    const uint8_t *uplane = src + dheight*sstride;
    const uint8_t *vplane = uplane + dheight*sstride/4;

    for (int i=0; i<dheight/2; i++) {
        const uint8_t *yrow1 = src + i*2*sstride;
        const uint8_t *yrow2 = src + i*2*sstride + sstride;
        const uint8_t *urow = uplane + i*sstride/2;
        const uint8_t *vrow = vplane + i*sstride/2;
        uint8_t *rgb1 = dest + i*2*dstride;
        uint8_t *rgb2 = dest + i*2*dstride + dstride;
        for (int j=0; j<dwidth/2; j++) {
            int cb = ((urow[j]-128) * 454)>>8;
            int cr = ((vrow[j]-128) * 359)>>8;
            int cg = ((vrow[j]-128) * 183 + (urow[j]-128) * 88)>>8;
            int r, g, b, yp;

            yp = yrow1[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*6 + 2] = MAX(0, MIN(255, r));
            rgb1[j*6 + 1] = MAX(0, MIN(255, g));
            rgb1[j*6 + 0] = MAX(0, MIN(255, b));

            yp = yrow1[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*6 + 5] = MAX(0, MIN(255, r));
            rgb1[j*6 + 4] = MAX(0, MIN(255, g));
            rgb1[j*6 + 3] = MAX(0, MIN(255, b));

            yp = yrow2[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*6 + 2] = MAX(0, MIN(255, r));
            rgb2[j*6 + 1] = MAX(0, MIN(255, g));
            rgb2[j*6 + 0] = MAX(0, MIN(255, b));

            yp = yrow2[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*6 + 5] = MAX(0, MIN(255, r));
            rgb2[j*6 + 4] = MAX(0, MIN(255, g));
            rgb2[j*6 + 3] = MAX(0, MIN(255, b));

        }
    }
    return 0;
}
int 
cam_pixel_convert_8u_yuv420p_to_8u_rgba(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    const uint8_t *uplane = src + dheight*sstride;
    const uint8_t *vplane = uplane + dheight*sstride/4;

    for (int i=0; i<dheight/2; i++) {
        const uint8_t *yrow1 = src + i*2*sstride;
        const uint8_t *yrow2 = src + i*2*sstride + sstride;
        const uint8_t *urow = uplane + i*sstride/2;
        const uint8_t *vrow = vplane + i*sstride/2;
        uint8_t *rgb1 = dest + i*2*dstride;
        uint8_t *rgb2 = dest + i*2*dstride + dstride;
        for (int j=0; j<dwidth/2; j++) {
            int cb = ((urow[j]-128) * 454)>>8;
            int cr = ((vrow[j]-128) * 359)>>8;
            int cg = ((vrow[j]-128) * 183 + (urow[j]-128) * 88)>>8;
            int r, g, b, yp;

            yp = yrow1[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*8 + 0] = MAX(0, MIN(255, r));
            rgb1[j*8 + 1] = MAX(0, MIN(255, g));
            rgb1[j*8 + 2] = MAX(0, MIN(255, b));
            rgb1[j*8 + 3] = 1;

            yp = yrow1[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*8 + 4] = MAX(0, MIN(255, r));
            rgb1[j*8 + 5] = MAX(0, MIN(255, g));
            rgb1[j*8 + 6] = MAX(0, MIN(255, b));
            rgb1[j*8 + 7] = 1;

            yp = yrow2[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*8 + 0] = MAX(0, MIN(255, r));
            rgb2[j*8 + 1] = MAX(0, MIN(255, g));
            rgb2[j*8 + 2] = MAX(0, MIN(255, b));
            rgb2[j*8 + 3] = 1;

            yp = yrow2[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*8 + 4] = MAX(0, MIN(255, r));
            rgb2[j*8 + 5] = MAX(0, MIN(255, g));
            rgb2[j*8 + 6] = MAX(0, MIN(255, b));
            rgb2[j*8 + 7] = 1;

        }
    }
    return 0;
}
int
cam_pixel_convert_8u_yuv420p_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    const uint8_t *uplane = src + dheight*sstride;
    const uint8_t *vplane = uplane + dheight*sstride/4;

    for (int i=0; i<dheight/2; i++) {
        const uint8_t *yrow1 = src + i*2*sstride;
        const uint8_t *yrow2 = src + i*2*sstride + sstride;
        const uint8_t *urow = uplane + i*sstride/2;
        const uint8_t *vrow = vplane + i*sstride/2;
        uint8_t *rgb1 = dest + i*2*dstride;
        uint8_t *rgb2 = dest + i*2*dstride + dstride;
        for (int j=0; j<dwidth/2; j++) {
            int cb = ((urow[j]-128) * 454)>>8;
            int cr = ((vrow[j]-128) * 359)>>8;
            int cg = ((vrow[j]-128) * 183 + (urow[j]-128) * 88)>>8;
            int r, g, b, yp;

            yp = yrow1[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*8 + 2] = MAX(0, MIN(255, r));
            rgb1[j*8 + 1] = MAX(0, MIN(255, g));
            rgb1[j*8 + 0] = MAX(0, MIN(255, b));
            rgb1[j*8 + 3] = 1;

            yp = yrow1[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb1[j*8 + 6] = MAX(0, MIN(255, r));
            rgb1[j*8 + 5] = MAX(0, MIN(255, g));
            rgb1[j*8 + 4] = MAX(0, MIN(255, b));
            rgb1[j*8 + 7] = 1;

            yp = yrow2[j*2];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*8 + 2] = MAX(0, MIN(255, r));
            rgb2[j*8 + 1] = MAX(0, MIN(255, g));
            rgb2[j*8 + 0] = MAX(0, MIN(255, b));
            rgb2[j*8 + 3] = 1;

            yp = yrow2[j*2 + 1];
            r = yp + cr;
            b = yp + cb;
            g = yp - cg;
            rgb2[j*8 + 6] = MAX(0, MIN(255, r));
            rgb2[j*8 + 5] = MAX(0, MIN(255, g));
            rgb2[j*8 + 4] = MAX(0, MIN(255, b));
            rgb2[j*8 + 7] = 1;
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_yuv420p_to_8u_gray(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i;
    for (i=0; i<dheight; i++) {
        memcpy (dest + i*dstride, src + i*sstride, dwidth);
    }
    return 0;
}

int 
cam_pixel_convert_8u_uyvy_to_8u_gray (uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i=0; i<dheight; i++) {
        uint8_t *drow = dest + i*dstride;
        const uint8_t *srow = src + i*sstride;
        for (j=0; j<dwidth; j++) {
            drow[j] = srow[j * 2 + 1];
        }
    }
    return 0;
}


int 
cam_pixel_convert_8u_uyvy_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth / 2; j++) {
            uint8_t u  = srow[4*j+0];
            uint8_t y1 = srow[4*j+1];
            uint8_t v  = srow[4*j+2];
            uint8_t y2 = srow[4*j+3];

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            drow[8*j+0] = MAX(0, MIN(255,b));
            drow[8*j+1] = MAX(0, MIN(255,g));
            drow[8*j+2] = MAX(0, MIN(255,r));
            drow[8*j+3] = 0;

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            drow[8*j+4] = MAX(0, MIN(255,b));
            drow[8*j+5] = MAX(0, MIN(255,g));
            drow[8*j+6] = MAX(0, MIN(255,r));
            drow[8*j+7] = 0;
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_uyvy_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth / 2; j++) {
            uint8_t u  = srow[4*j+0];
            uint8_t y1 = srow[4*j+1];
            uint8_t v  = srow[4*j+2];
            uint8_t y2 = srow[4*j+3];

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            drow[6*j+0] = MAX(0, MIN(255,r));
            drow[6*j+1] = MAX(0, MIN(255,g));
            drow[6*j+2] = MAX(0, MIN(255,b));

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            drow[6*j+3] = MAX(0, MIN(255,r));
            drow[6*j+4] = MAX(0, MIN(255,g));
            drow[6*j+5] = MAX(0, MIN(255,b));
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_yuyv_to_8u_gray (uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;
    for (i=0; i<dheight; i++) {
        uint8_t *drow = dest + i*dstride;
        const uint8_t *srow = src + i*sstride;
        for (j=0; j<dwidth; j++) {
            drow[j] = srow[j * 2];
        }
    }
    return 0;
}


int 
cam_pixel_convert_8u_yuyv_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth / 2; j++) {
            uint8_t y1 = srow[4*j+0];
            uint8_t u  = srow[4*j+1];
            uint8_t y2 = srow[4*j+2];
            uint8_t v  = srow[4*j+3];

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            drow[8*j+0] = MAX(0, MIN(255,b));
            drow[8*j+1] = MAX(0, MIN(255,g));
            drow[8*j+2] = MAX(0, MIN(255,r));
            drow[8*j+3] = 0;

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            drow[8*j+4] = MAX(0, MIN(255,b));
            drow[8*j+5] = MAX(0, MIN(255,g));
            drow[8*j+6] = MAX(0, MIN(255,r));
            drow[8*j+7] = 0;
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_yuyv_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth / 2; j++) {
            uint8_t y1 = srow[4*j+0];
            uint8_t u  = srow[4*j+1];
            uint8_t y2 = srow[4*j+2];
            uint8_t v  = srow[4*j+3];

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            drow[6*j+0] = MAX(0, MIN(255,r));
            drow[6*j+1] = MAX(0, MIN(255,g));
            drow[6*j+2] = MAX(0, MIN(255,b));

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            drow[6*j+3] = MAX(0, MIN(255,r));
            drow[6*j+4] = MAX(0, MIN(255,g));
            drow[6*j+5] = MAX(0, MIN(255,b));
        }
    }
    return 0;
}

int
cam_pixel_convert_8u_iyu1_to_8u_gray (uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j, k;
    for (i=0; i<dheight; i++) {
        uint8_t *drow = dest + i*dstride;
        const uint8_t *srow = src + i*sstride;
        for (j=0, k=0; j<dwidth; j++, k++) {
            if ((k%3) == 0) k++;
            drow[j] = srow[k];
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_iyu1_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth / 4; j++) {
            uint8_t u  = srow[6*j+0];
            uint8_t y1 = srow[6*j+1];
            uint8_t y2 = srow[6*j+2];
            uint8_t v  = srow[6*j+3];
            uint8_t y3 = srow[6*j+4];
            uint8_t y4 = srow[6*j+5];

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            drow[12*j+0]  = MAX(0, MIN(255,r));
            drow[12*j+1]  = MAX(0, MIN(255,g));
            drow[12*j+2]  = MAX(0, MIN(255,b));

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            drow[12*j+3]  = MAX(0, MIN(255,r));
            drow[12*j+4]  = MAX(0, MIN(255,g));
            drow[12*j+5]  = MAX(0, MIN(255,b));

            r = y3 + cr;
            b = y3 + cb;
            g = y3 - cg;
            drow[12*j+6]  = MAX(0, MIN(255,r));
            drow[12*j+7]  = MAX(0, MIN(255,g));
            drow[12*j+8]  = MAX(0, MIN(255,b));

            r = y4 + cr;
            b = y4 + cb;
            g = y4 - cg;
            drow[12*j+9]  = MAX(0, MIN(255,r));
            drow[12*j+10] = MAX(0, MIN(255,g));
            drow[12*j+11] = MAX(0, MIN(255,b));
        }
    }
    return 0;
}

int 
cam_pixel_convert_8u_iyu1_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride)
{
    int i, j;

    for (i = 0; i < dheight; i++) {
        uint8_t * drow = dest + i * dstride;
        const uint8_t * srow = src + i * sstride;
        for (j = 0; j < dwidth / 4; j++) {
            uint8_t u  = srow[6*j+0];
            uint8_t y1 = srow[6*j+1];
            uint8_t y2 = srow[6*j+2];
            uint8_t v  = srow[6*j+3];
            uint8_t y3 = srow[6*j+4];
            uint8_t y4 = srow[6*j+5];

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = y1 + cr;
            b = y1 + cb;
            g = y1 - cg;
            drow[16*j+0]  = MAX(0, MIN(255,b));
            drow[16*j+1]  = MAX(0, MIN(255,g));
            drow[16*j+2]  = MAX(0, MIN(255,r));
            drow[16*j+3]  = 0;

            r = y2 + cr;
            b = y2 + cb;
            g = y2 - cg;
            drow[16*j+4]  = MAX(0, MIN(255,b));
            drow[16*j+5]  = MAX(0, MIN(255,g));
            drow[16*j+6]  = MAX(0, MIN(255,r));
            drow[16*j+7]  = 0;

            r = y3 + cr;
            b = y3 + cb;
            g = y3 - cg;
            drow[16*j+8]  = MAX(0, MIN(255,b));
            drow[16*j+9]  = MAX(0, MIN(255,g));
            drow[16*j+10] = MAX(0, MIN(255,r));
            drow[16*j+11] = 0;

            r = y4 + cr;
            b = y4 + cb;
            g = y4 - cg;
            drow[16*j+12] = MAX(0, MIN(255,b));
            drow[16*j+13] = MAX(0, MIN(255,g));
            drow[16*j+14] = MAX(0, MIN(255,r));
            drow[16*j+15] = 0;
        }
    }
    return 0;
}

int
cam_pixel_replicate_border_8u (uint8_t * src, int sstride, int width, int height)
{

    memcpy (src - sstride, src, width);
    memcpy (src + height * sstride, src + (height-1)*sstride, width);

    int i;
    for (i = -1; i < height+1; i++) {
        src[i*sstride-1] = src[i*sstride];
        src[i*sstride + width] = src[i*sstride + width - 1];
    }
    return 0;
}

int
cam_pixel_replicate_bayer_border_8u (uint8_t * src, int sstride, int width,
        int height)
{

    memcpy (src - 2*sstride, src, width);
    memcpy (src - sstride, src + sstride, width);

    memcpy (src + (height+1)*sstride, src + (height-1)*sstride, width);
    memcpy (src + height*sstride, src + (height-2)*sstride, width);

    int i;
    for (i = -2; i < height+2; i++) {
        src[i*sstride-2] = src[i*sstride];
        src[i*sstride-1] = src[i*sstride+1];
        src[i*sstride + width + 1] = src[i*sstride + width - 1];
        src[i*sstride + width] = src[i*sstride + width - 2];
    }
    return 0;
}

int
cam_pixel_split_bayer_planes_8u (uint8_t *dst[4], int dstride,
        const uint8_t * src, int sstride, int width, int height)
{
    if (!cpuid_detected) {
        cpuid_detect (&has_sse2, &has_sse3);
        cpuid_detected = 1;
    }

#ifdef HAVE_INTEL
    if (has_sse2)
        return cam_pixel_split_bayer_planes_8u_sse2 (dst, dstride,
                src, sstride, width, height);
#endif

    fprintf (stderr, "Error: cam_pixel_split_bayer_planes_8u requires at "
            "least SSE2 support\n");
    return -1;
}

int
cam_pixel_bayer_interpolate_to_8u_bgra (uint8_t ** src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format)
{
    if (!cpuid_detected) {
        cpuid_detect (&has_sse2, &has_sse3);
        cpuid_detected = 1;
    }
    
#ifdef HAVE_INTEL
    if (has_sse3)
        return cam_pixel_bayer_interpolate_to_8u_bgra_sse3 (src, sstride,
                dst, dstride, width, height, format);
    if (has_sse2)
        return cam_pixel_bayer_interpolate_to_8u_bgra_sse2 (src, sstride,
                dst, dstride, width, height, format);
#endif

    fprintf (stderr, "Error: cam_pixel_bayer_interpolate_to_8u_bgra "
            "requires at least SSE2 support\n");
    return -1;
}

int
cam_pixel_bayer_interpolate_to_8u_gray (uint8_t * src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format)
{
    if (!cpuid_detected) {
        cpuid_detect (&has_sse2, &has_sse3);
        cpuid_detected = 1;
    }

#ifdef HAVE_INTEL
    if (has_sse3)
        return cam_pixel_bayer_interpolate_to_8u_gray_sse3 (src, sstride,
                dst, dstride, width, height, format);
    if (has_sse2)
        return cam_pixel_bayer_interpolate_to_8u_gray_sse2 (src, sstride,
                dst, dstride, width, height, format);
#endif

    fprintf (stderr, "Error: cam_pixel_bayer_interpolate_to_8u_gray "
            "requires at least SSE2 support\n");
    return -1;
}

int 
cam_pixel_convert_bayer_to_8u_bgra (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride, CamPixelFormat format)
{
    if (format != CAM_PIXEL_FORMAT_BAYER_BGGR &&
        format != CAM_PIXEL_FORMAT_BAYER_GRBG &&
        format != CAM_PIXEL_FORMAT_BAYER_GBRG &&
        format != CAM_PIXEL_FORMAT_BAYER_RGGB) {
        fprintf (stderr, "%s:%d:%s invalid pixel format %d\n", 
                __FILE__, __LINE__, __FUNCTION__, format);
        return -1;
    }

    /* ensure stride is 16-byte aligned and add 32 extra bytes for the
     * border padding */
    void *bayer_planes[4];
    int plane_stride = ((width + 0xf)&(~0xf)) + 32;
    for (int i = 0; i < 4; i++) {
        bayer_planes[i] = MALLOC_ALIGNED (plane_stride * (height + 2));
    }

    // alocate a 16-byte aligned buffer for the interpolated image
    int bgra_stride = width*4;
    void *bgra_img = MALLOC_ALIGNED (height * bgra_stride);

    // allocate a 16-byte aligned buffer for the source image
    int bayer_stride = width;
    const void* bayer_img = src;
    void* tmp_buf = NULL;
    if(!CAM_IS_ALIGNED16(src)) {
        tmp_buf = MALLOC_ALIGNED (height * bayer_stride);
        // copy the source image into the 16-byte aligned buffer
        cam_pixel_copy_8u_generic (src, sstride, 
                tmp_buf, bayer_stride,
                0, 0, 0, 0, width, height, 8);
        bayer_img = tmp_buf;
    }

    // split the bayer image 
    uint8_t * planes[] = {
        bayer_planes[0] + plane_stride + 16,
        bayer_planes[1] + plane_stride + 16,
        bayer_planes[2] + plane_stride + 16,
        bayer_planes[3] + plane_stride + 16,
    };
    int p_width = width / 2;
    int p_height = height / 2;

    cam_pixel_split_bayer_planes_8u (planes, plane_stride,
            bayer_img, bayer_stride, p_width, p_height);
    for (int j = 0; j < 4; j++)
        cam_pixel_replicate_border_8u (planes[j], plane_stride, p_width, p_height);

    // interpolate
    cam_pixel_bayer_interpolate_to_8u_bgra (planes, plane_stride,
            bgra_img, bgra_stride, 
            width, height, format);

    // copy to destination
    cam_pixel_copy_8u_generic (bgra_img, bgra_stride,
            dest, dstride, 0, 0, 0, 0, width, height, 8 * 4);

    // release allocated memory
    free (tmp_buf);
    free (bgra_img);
    for (int i=0; i<4; i++) {
        free (bayer_planes[i]);
    }

    return 0;
}

int 
cam_pixel_convert_bayer_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride, CamPixelFormat format)
{
    if (format != CAM_PIXEL_FORMAT_BAYER_BGGR &&
        format != CAM_PIXEL_FORMAT_BAYER_GRBG &&
        format != CAM_PIXEL_FORMAT_BAYER_GBRG &&
        format != CAM_PIXEL_FORMAT_BAYER_RGGB) {
        fprintf (stderr, "%s:%d:%s invalid pixel format %d\n", 
                __FILE__, __LINE__, __FUNCTION__, format);
        return -1;
    }

    int plane_stride = ((width + 0xf)&(~0xf)) + 32;
    void *plane_buf = MALLOC_ALIGNED (plane_stride * (height + 4));
    uint8_t *plane = (uint8_t*) plane_buf +  2 * plane_stride + 16;

    // copy the source image into the 16-byte aligned buffer
    cam_pixel_copy_8u_generic (src, sstride, 
            plane, plane_stride,
            0, 0, 0, 0, width, height, 8);

    cam_pixel_replicate_border_8u (plane, plane_stride, width, height);

    if (!CAM_IS_ALIGNED16 (dest) || !CAM_IS_ALIGNED16 (dstride)) {
        void *gray_buf = MALLOC_ALIGNED (height * plane_stride);

        // interpolate
        cam_pixel_bayer_interpolate_to_8u_gray (plane, plane_stride,
                gray_buf, plane_stride, width, height, format);

        cam_pixel_copy_8u_generic (gray_buf, plane_stride, 
                dest, dstride,
                0, 0, 0, 0, width, height, 8);

        // release allocated memory
        free (gray_buf);
    } else {
        cam_pixel_bayer_interpolate_to_8u_gray (plane, plane_stride,
                dest, dstride, width, height, format);
    }
    free (plane_buf);

    return 0;
}

int 
cam_pixel_copy_8u_generic (const uint8_t *src, int sstride, 
        uint8_t *dst, int dstride, 
        int src_x, int src_y, 
        int dst_x, int dst_y,
        int width, int height,
        int bits_per_pixel)
{
    if (bits_per_pixel % 8) return -1;
    int bytes_per_pixel = bits_per_pixel / 8;

    int i;
    for (i=0; i<height; i++) {
        uint8_t *dst_row = dst + (dst_y + i) * dstride;
        const uint8_t *src_row = src + (src_y + i) * sstride;

        memcpy (dst_row + dst_x * bytes_per_pixel, 
                src_row + src_x * bytes_per_pixel, 
                width * bytes_per_pixel);
    }
    return 0;
}

#if 0
int
cam_pixel_split_2_planes_8u (uint8_t * dst1, int dstride1, uint8_t * dst2,
        int dstride2, uint8_t * src, int sstride, int width, int height)
{
    __m128i mask;
    int i, j;

    if (!CAM_IS_ALIGNED16(dst1) || !CAM_IS_ALIGNED16(dstride1)) {
        fprintf (stderr, "cam_pixel_split_2_planes_8u: dst1 is not 16-byte aligned\n");
        return -1;
    }
    if (!CAM_IS_ALIGNED16(dst2) || !CAM_IS_ALIGNED16(dstride2)) {
        fprintf (stderr, "cam_pixel_split_2_planes_8u: dst2 is not 16-byte aligned\n");
        return -1;
    }
    if (!CAM_IS_ALIGNED16(src) || !CAM_IS_ALIGNED32(sstride)) {
        fprintf (stderr, "cam_pixel_split_2_planes_8u: src is not 32-byte aligned\n");
        return -1;
    }

    mask = _mm_set1_epi16 (0xff);
    for (i = 0; i < height; i++) {
        uint8_t * drow1 = dst1 + i * dstride1;
        uint8_t * drow2 = dst2 + i * dstride2;
        uint8_t * srow = src + i * sstride;
        for (j = 0; j < width; j += 16) {
            __m128i s1, s2, t1, t2, out;
            s1 = _mm_load_si128 ((__m128i *)(srow + 2*j));
            s2 = _mm_load_si128 ((__m128i *)(srow + 2*j + 16));

            t1 = _mm_and_si128 (s1, mask);
            t2 = _mm_and_si128 (s2, mask);

            out = _mm_packus_epi16 (t1, t2);
            _mm_store_si128 ((__m128i *)(drow1 + j), out);

            t1 = _mm_srli_epi16 (s1, 8);
            t2 = _mm_srli_epi16 (s2, 8);

            out = _mm_packus_epi16 (t1, t2);
            _mm_store_si128 ((__m128i *)(drow2 + j), out);
        }
    }
    return 0;
}

int
pixel_join_2_planes_8u (uint8_t * dest, int dstride, uint8_t * src1,
        int sstride1, uint8_t * src2, int sstride2, int width, int height)
{
    int i, j;

    if (!CAM_IS_ALIGNED16(dest) || !CAM_IS_ALIGNED32(dstride)) {
        fprintf (stderr, "pixel_join_2_planes_8u: dest is not 32-byte aligned\n");
        return -1;
    }
    if (!CAM_IS_ALIGNED16(src1) || !CAM_IS_ALIGNED16(sstride1)) {
        fprintf (stderr, "pixel_join_2_planes_8u: src1 is not 16-byte aligned\n");
        return -1;
    }
    if (!CAM_IS_ALIGNED16(src2) || !CAM_IS_ALIGNED16(sstride2)) {
        fprintf (stderr, "pixel_join_2_planes_8u: src2 is not 16-byte aligned\n");
        return -1;
    }

    for (i = 0; i < height; i++) {
        uint8_t * drow = dest + i * dstride;
        uint8_t * srow1 = src1 + i * sstride1;
        uint8_t * srow2 = src2 + i * sstride2;
        for (j = 0; j < width; j += 16) {
            __m128i s1, s2, d1, d2;
            s1 = _mm_load_si128 ((__m128i *)(srow1 + j));
            s2 = _mm_load_si128 ((__m128i *)(srow2 + j));

            d1 = _mm_unpacklo_epi8 (s1, s2);
            d2 = _mm_unpackhi_epi8 (s1, s2);

            _mm_store_si128 ((__m128i *)(drow + 2*j), d1);
            _mm_store_si128 ((__m128i *)(drow + 2*j + 16), d2);
        }
    }
    return 0;
}
#endif
