#ifndef _jpeg_utils_ijg_h_
#define _jpeg_utils_ijg_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int
jpegijg_decompress_8u_rgb (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride);
int
jpegijg_decompress_8u_gray (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride);

int
jpegijg_compress_8u_gray (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality);
int
jpegijg_compress_8u_rgb (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality);
int
jpegijg_compress_8u_bgra (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality);

#ifdef __cplusplus
}
#endif


#endif
