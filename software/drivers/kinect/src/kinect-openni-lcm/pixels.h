#ifndef __CAM_PIXELS_H__
#define __CAM_PIXELS_H__

#include <stdint.h>

/**
 * SECTION:pixels
 * @short_description: Pixel format definitions and colorspace conversion routines.
 */
#define cam_pf_fourcc(a,b,c,d)\
   (((uint32_t)(a))     | \
    ((uint32_t)(b)<<8)  | \
    ((uint32_t)(c)<<16) | \
    ((uint32_t)(d)<<24))

#define CAM_IS_ALIGNED16(x) (((uintptr_t)(x) & 0xf) == 0)
#define CAM_IS_ALIGNED32(x) (((uintptr_t)(x) & 0x1f) == 0)
#define CAM_IS_ALIGNED128(x) (((uintptr_t)(x) & 0x7f) == 0)

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* standard FourCCs */
    CAM_PIXEL_FORMAT_UYVY=cam_pf_fourcc('U','Y','V','Y'),  /* YUV 4:2:2 */
    CAM_PIXEL_FORMAT_YUYV=cam_pf_fourcc('Y','U','Y','V'),  /* YUV 4:2:2 reverse byte 
                                                      ordering (same as YUY2)*/
    CAM_PIXEL_FORMAT_IYU1=cam_pf_fourcc('I','Y','U','1'),  /* YUV 4:1:1 */
    CAM_PIXEL_FORMAT_IYU2=cam_pf_fourcc('I','Y','U','2'),  /* YUV 4:4:4 */
    CAM_PIXEL_FORMAT_YUV420=cam_pf_fourcc('Y','U','1','2'), /* YUV 4:2:0 */
    CAM_PIXEL_FORMAT_YUV411P=cam_pf_fourcc('4','1','1','P'), /* YUV 4:1:1 planar */
//    CAM_PIXEL_FORMAT_YV12=cam_pf_fourcc('Y','V','1','2'),  /* planar YUV 4:2:0, 
//                                                      Y - V - U */
    CAM_PIXEL_FORMAT_I420=cam_pf_fourcc('I','4','2','0'),  /* planar YUV 4:2:0, 
                                                      Y - U - V */
    CAM_PIXEL_FORMAT_NV12=cam_pf_fourcc('N','V','1','2'), /* YUV 4:2:0, 
                                                     Y plane, followed by 
                                                     interleaved U-V plane */

    CAM_PIXEL_FORMAT_GRAY=cam_pf_fourcc('G','R','E','Y'),
    CAM_PIXEL_FORMAT_RGB=cam_pf_fourcc('R','G','B','3'),
    CAM_PIXEL_FORMAT_BGR=cam_pf_fourcc('B','G','R','3'),
    CAM_PIXEL_FORMAT_RGBA=cam_pf_fourcc('R','G','B','4'),
    CAM_PIXEL_FORMAT_BGRA=cam_pf_fourcc('B','G','R','4'),

    CAM_PIXEL_FORMAT_BAYER_BGGR=cam_pf_fourcc('B','A','8','1'),
    CAM_PIXEL_FORMAT_BAYER_GBRG=cam_pf_fourcc('X','X','X','2'),
    CAM_PIXEL_FORMAT_BAYER_GRBG=cam_pf_fourcc('X','X','X','3'),
    CAM_PIXEL_FORMAT_BAYER_RGGB=cam_pf_fourcc('X','X','X','4'),

    /* Others */
    CAM_PIXEL_FORMAT_BE_BAYER16_BGGR=cam_pf_fourcc('B','B','A','1'), /* 16-bpp, bayer, big-endian */
    CAM_PIXEL_FORMAT_BE_BAYER16_GBRG=cam_pf_fourcc('B','B','A','2'),
    CAM_PIXEL_FORMAT_BE_BAYER16_GRBG=cam_pf_fourcc('B','B','A','3'),
    CAM_PIXEL_FORMAT_BE_BAYER16_RGGB=cam_pf_fourcc('B','B','A','4'),

    CAM_PIXEL_FORMAT_LE_BAYER16_BGGR=cam_pf_fourcc('L','B','A','1'), /* 16-bpp, bayer, little-endian */
    CAM_PIXEL_FORMAT_LE_BAYER16_GBRG=cam_pf_fourcc('L','B','A','2'),
    CAM_PIXEL_FORMAT_LE_BAYER16_GRBG=cam_pf_fourcc('L','B','A','3'),
    CAM_PIXEL_FORMAT_LE_BAYER16_RGGB=cam_pf_fourcc('L','B','A','4'),

    CAM_PIXEL_FORMAT_BE_RGB16=358,          /* 48-bpp rgb (16-bits per channel), big-endian */
    CAM_PIXEL_FORMAT_LE_RGB16=cam_pf_fourcc('R','G','B','L'), /* 48-bpp rgb (16-bits per channel), little-endian */

    CAM_PIXEL_FORMAT_BE_GRAY16=357,         /* 16-bpp luminance/grayscale, big-endian */
    CAM_PIXEL_FORMAT_LE_GRAY16=cam_pf_fourcc('L','G','1','6'), /* 16-bit grayscale, little-endian */

    CAM_PIXEL_FORMAT_MJPEG=cam_pf_fourcc('M','J','P','G'),
    CAM_PIXEL_FORMAT_BE_SIGNED_GRAY16=359,
    CAM_PIXEL_FORMAT_BE_SIGNED_RGB16=360,

    CAM_PIXEL_FORMAT_FLOAT_GRAY32=cam_pf_fourcc('F','G','3','2'), /* 32-bit grayscale IEEE float, native byte order */
//    CAM_PIXEL_FORMAT_FLOAT_RGB32,
    CAM_PIXEL_FORMAT_INVALID=0xFFFFFFFE,
    CAM_PIXEL_FORMAT_ANY=0xFFFFFFFF,
} CamPixelFormat;

/**
 * cam_pixel_format_nickname:
 *
 * Returns: a short, descriptive string that indicates the meaning of
 * CamPixelFormat @p, useful for debugging and UI elements.
 */
const char * cam_pixel_format_nickname (CamPixelFormat p);

/**
 * cam_pixel_format_bpp:
 *
 * Returns: the number of bits per pixel of a given CamPixelFormat.  Note that
 * if the format has multiple color components per pixel, the returned
 * value is the total bits per pixel, not per component.
 */
int cam_pixel_format_bpp (CamPixelFormat p);

/**
 * cam_pixel_format_stride_meaningful:
 *
 * Returns: 1 if rowstride is meaningful for the pixelformat, 0 if not.
 */
int cam_pixel_format_stride_meaningful (CamPixelFormat p);

/**
 * cam_pixel_convert_8u_gray_to_64f_gray:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Converts an image composed of 8-bit unsigned pixels to an image composed
 * of double-precision pixels.  Pixel values are rescaled to [0,1].
 * This function works on multiple-channel images, as long as the width
 * and height are modified accordingly.
 */
int cam_pixel_convert_8u_gray_to_64f_gray (double * dest, int dstride,
        int dwidth, int dheight, const uint8_t * src, int sstride);

int cam_pixel_convert_8u_gray_to_32f_gray (float *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);

int cam_pixel_convert_32f_gray_to_8u_gray (uint8_t *dest, int dstride,
        int dwidth, int dheight, const float *src, int sstride);

/**
 * cam_pixel_convert_8u_gray_to_8u_RGB:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Converts a grayscale image composed of 8-bit unsigned pixels to an
 * RGB image (3 bytes per pixel) by duplicating the grayscale value across
 * the three color channels.
 */
int cam_pixel_convert_8u_gray_to_8u_RGB (uint8_t * dest, int dstride,
        int dwidth, int dheight, const uint8_t * src, int sstride);

/**
 * cam_pixel_convert_8u_gray_to_8u_RGBA:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Converts a grayscale image composed of 8-bit unsigned pixels to an
 * RGBA image (4 bytes per pixel) by duplicating the grayscale value across
 * the three color channels and setting the alpha to 255.
 */
int cam_pixel_convert_8u_gray_to_8u_RGBA (uint8_t * dest, int dstride,
        int dwidth, int dheight, const uint8_t * src, int sstride);

/**
 * cam_pixel_apply_lut_8u:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 * @lut: The lookup table (LUT) pre-allocated and populated by the caller.
 *
 * Apply a lookup table to the pixel values of a source image and write
 * the result to a destination image.  The LUT is an array of length
 * 256 that contains the output pixel value indexed by input pixel value.
 */
int cam_pixel_apply_lut_8u (uint8_t * dest, int dstride, int dwidth, int dheight,
        const uint8_t * src, int sstride, const uint8_t * lut);

/**
 * cam_pixel_convert_8u_rgb_to_8u_bgr:
 * @dst: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @width: Width of the destination image in pixels.
 * @height: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Reverses the order of channels in a 3-channel image.
 */
int cam_pixel_convert_8u_rgb_to_8u_bgr(uint8_t *dst, int dstride, int width, 
        int height, const uint8_t *src, int sstride);

int cam_pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride);

/**
 * cam_pixel_convert_8u_rgb_to_32f_gray:
 * @dst: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @width: Width of the destination image in pixels.
 * @height: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Converts a 3-channel RGB image to 32-bit floating point grayscale.
 * Resulting pixel values are in the range [0,1]
 */
int cam_pixel_convert_8u_rgb_to_32f_gray (float *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride);

/**
 * cam_pixel_convert_8u_bgr_to_8u_rgb:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Reverses the order of channels in a 3-channel image.
 */
int cam_pixel_convert_8u_bgr_to_8u_rgb(uint8_t *dest, int dstride, int dwidth, 
        int dheight, const uint8_t *src, int sstride);

int cam_pixel_convert_8u_rgb_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);

/**
 * cam_pixel_convert_8u_bgra_to_8u_bgr:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Converts a 4-channel image to a 3-channel image by removing the 4th
 * channel.
 */
int cam_pixel_convert_8u_bgra_to_8u_bgr(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);

/**
 * cam_pixel_convert_8u_bgra_to_8u_bgr:
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer.
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * Converts a 4-channel image to a 3-channel image by removing the 4th
 * channel.
 */
int cam_pixel_convert_8u_bgra_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);

/**
 * cam_pixel_convert_8u_yuv420p_to_8u_rgb
 * @dest: The destination buffer pre-allocated by the caller.
 * @dstride: Number of bytes between the start of each image row in the
 *      destination buffer. TODO
 * @dwidth: Width of the destination image in pixels.
 * @dheight: Height of the destination image in pixels.
 * @src: The source image.
 * @sstride: Number of bytes between the start of each image row in the
 *      source buffer.
 *
 * TODO
 */
int cam_pixel_convert_8u_yuv420p_to_8u_rgb(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_yuv420p_to_8u_rgba(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_yuv420p_to_8u_bgr(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_yuv420p_to_8u_bgra(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_yuv420p_to_8u_gray(uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride);

int cam_pixel_convert_8u_uyvy_to_8u_gray (uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_uyvy_to_8u_bgra(uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_uyvy_to_8u_rgb(uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);

int cam_pixel_convert_8u_yuyv_to_8u_gray (uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_yuyv_to_8u_bgra(uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_yuyv_to_8u_rgb(uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);

int cam_pixel_convert_8u_iyu1_to_8u_gray (uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_iyu1_to_8u_bgra(uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
int cam_pixel_convert_8u_iyu1_to_8u_rgb(uint8_t *dest, int dstride,
        int dwidth, int dheight, const uint8_t *src, int sstride);
/**
 * cam_pixel_replicate_border_8u:
 * @src: Pointer to the top-left pixel of the input image.  The output
 *     is written in-place, and thus the input buffer must be large enough
 *     to accomodate an image 2 pixels wider and higher than the input image.
 * @sstride: Number of bytes between the start of each image row in the
 *     input/output buffer.  Must be larger than @width to accomodate
 *     duplicated columns.
 * @width: Width of the input image.  Output image will have width @width+2.
 * @height: Height of the input image.  Output image will have height
 *     @height+2.
 *
 * Duplicates the 1-pixel thick border around the image by copying it to
 * the next outer row/column of pixels.  @src points to the top-left pixel
 * in the input image, but the buffer must exist for negative offsets to
 * @src to allow room for the duplicated pixels.  There also must be
 * sufficient unused space between rows for the duplicated column.
 */
int cam_pixel_replicate_border_8u (uint8_t * src, int sstride, int width,
        int height);

/**
 * cam_pixel_replicate_bayer_border_8u:
 * @src: Pointer to the top-left pixel of the input image.  The output
 *     is written in-place, and thus the input buffer must be large enough
 *     to accomodate an image 4 pixels wider and higher than the input image.
 * @sstride: Number of bytes between the start of each image row in the
 *     input/output buffer.  Must be larger than @width to accomodate
 *     duplicated columns.
 * @width: Width of the input image.  Output image will have width @width+4.
 * @height: Height of the input image.  Output image will have height
 *     @height+4.
 *
 * Duplicates the 2-pixel thick border around the image by copying it to
 * the next outer rows/columns of pixels.  @src points to the top-left pixel
 * in the input image, but the buffer must exist for negative offsets to
 * @src to allow room for the duplicated pixels.  There also must be
 * sufficient unused space between rows for the duplicated column.  This
 * function is intended to preserve any 2-pixel wide bayer pattern that
 * exists in the image.
 */
int cam_pixel_replicate_bayer_border_8u (uint8_t * src, int sstride, int width,
        int height);

/**
 * cam_pixel_split_bayer_planes_8u:
 * @dst: Array of length 4 that contains destination pointers for the 4
 *     output planes.  Within the 2x2 bayer pattern, dst[0] is given the
 *     top-left set of pixels, dst[1] the top-right, dst[2] the bottom-left,
 *     and dst[3] the bottom-right.  The start of each buffer must be
 *     16-byte aligned.
 * @dstride: Number of bytes between the start of each row in the output
 *     buffers.  Each output buffer must have the same stride and it must
 *     be a multiple of 16 bytes.
 * @src: The source image.  Must be 16-byte aligned.
 * @sstride: Number of bytes between the start of each row in the input
 *     image.  Must be a multiple of 16 bytes.
 * @width: Width of each output plane.
 * @height: Height of each output plane.
 *
 * Splits the R, B, Gb, and Gr components of a bayer-patterned image into
 * four separate planes.  This function is SSE2 accelerated.
 */
int cam_pixel_split_bayer_planes_8u (uint8_t *dst[4], int dstride,
        const uint8_t * src, int sstride, int width, int height);

/**
 * cam_pixel_bayer_interpolate_to_8u_bgra:
 * @src: An array of length 4 that contains the pointers to the 4
 *     source image planes.  These are probably obtained via
 *     cam_pixel_split_bayer_planes_8u().  If an image of the same resolution
 *     of the input image is desired, the border of each source plane
 *     must be duplicated with cam_pixel_replicate_border_8u().
 * @sstride: Stride in bytes of each source image.
 * @dst: Destination image buffer.
 * @dstride: Stride in bytes of destination image.
 * @width: Width in pixels of output image.
 * @height: Height in pixels of output image.
 * @format: Pixel format of the bayer-patterned image.  Must be one of
 *     the four 8u bayer pattern pixel formats.
 *
 * Performs bayer interpolation on an image and produces a 4-channel BGRA
 * image where alpha is set to 255.  The method used is described by:
 *
 * H.S. Malvar, L. He, and R. Cutler. "High-quality linear interpolation
 * for demosaicing of Bayer-patterned color images".  In Proc. IEEE
 * ICASSP 2004.  May 2004.  pp. 485-8.
 *
 * This function is SSE2/SSE3 accelerated.
 */
int cam_pixel_bayer_interpolate_to_8u_bgra (uint8_t ** src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format);

/**
 * cam_pixel_bayer_interpolate_to_8u_gray:
 * @src: The source bayer-patterned image.  If an image of the same
 *     resolution of the same resolution as the input image is desired,
 *     the border of the source image must be duplicated with
 *     cam_pixel_replicate_bayer_border_8u().
 * @sstride: Stride in bytes of the source image.
 * @dst: Destination image buffer.
 * @dstride: Stride in bytes of destination image.
 * @width: Width in pixels of output image.
 * @height: Height in pixels of output image.
 * @format: Pixel format of the bayer-patterned image.  Must be one of
 *     the four 8u bayer pattern pixel formats.
 *
 * Performs bayer interpolation on an image and sums the red, green, and
 * blue channels with weights 0.25, 0.50, and 0.25, respectively, to
 * produce a 1-channel grayscale image.  The method used is described by:
 *
 * H.S. Malvar, L. He, and R. Cutler. "High-quality linear interpolation
 * for demosaicing of Bayer-patterned color images".  In Proc. IEEE
 * ICASSP 2004.  May 2004.  pp. 485-8.
 *
 * This function is SSE2/SSE3 accelerated.
 */
int cam_pixel_bayer_interpolate_to_8u_gray (uint8_t * src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format);

/**
 * cam_pixel_convert_bayer_to_8u_bgra:
 *
 * convenience function to perform bayer interpolation.  Thin wrapper around
 * cam_pixel_bayer_interpolate_to_8u_bgra that does not require the source or
 * destination buffers to be 16-byte aligned.
 */
int cam_pixel_convert_bayer_to_8u_bgra (uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride, CamPixelFormat format);

/**
 * cam_pixel_convert_bayer_to_8u_gray:
 *
 * convenience function to perform bayer interpolation.  Thin wrapper around
 * cam_pixel_bayer_interpolate_to_8u_gray that does not require the source or
 * destination buffers to be 16-byte aligned.
 */
int cam_pixel_convert_bayer_to_8u_gray (uint8_t *dest, int dstride, int dwidth,
        int dheight, const uint8_t *src, int sstride, CamPixelFormat format);

int cam_pixel_copy_8u_generic (const uint8_t *src, int sstride, 
        uint8_t *dst, int dstride, 
        int src_x, int src_y, 
        int dst_x, int dst_y,
        int width, int height,
        int bits_per_pixel);

#ifdef __cplusplus
}
#endif

#endif
