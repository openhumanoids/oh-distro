#include <stdio.h>
#include <stdint.h>
#include <emmintrin.h>

#include "pixels_sse2.h"

int
cam_pixel_split_bayer_planes_8u_sse2 (uint8_t *dst[4], int dstride,
        const uint8_t * src, int sstride, int width, int height)
{
    __m128i mask;
    int i, j;

    for (i = 0; i < 4; i++) {
        if (!CAM_IS_ALIGNED16(dst[i]) || !CAM_IS_ALIGNED16(dstride)) {
            fprintf (stderr, "cam_pixel_split_bayer_planes_8u: dst[%d] is not "
                    "16-byte aligned\n", i);
            return -1;
        }
    }
    if (!CAM_IS_ALIGNED16(src) || !CAM_IS_ALIGNED16(sstride)) {
        fprintf (stderr, "cam_pixel_split_bayer_planes_8u: src is not 16-byte "
                "aligned\n");
        return -1;
    }

    /* If the source stride is not a multiple of 32 pixels, we need to
     * fix up the last column at the end. */
    if (!CAM_IS_ALIGNED32 (sstride))
        width -= 8;

    mask = _mm_set1_epi16 (0xff);
    for (i = 0; i < height; i++) {
        uint8_t * drow1 = dst[0] + i * dstride;
        uint8_t * drow2 = dst[1] + i * dstride;
        uint8_t * drow3 = dst[2] + i * dstride;
        uint8_t * drow4 = dst[3] + i * dstride;
        const uint8_t * srow = src + 2*i*sstride;
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

            s1 = _mm_load_si128 ((__m128i *)(srow + sstride + 2*j));
            s2 = _mm_load_si128 ((__m128i *)(srow + sstride + 2*j + 16));

            t1 = _mm_and_si128 (s1, mask);
            t2 = _mm_and_si128 (s2, mask);

            out = _mm_packus_epi16 (t1, t2);
            _mm_store_si128 ((__m128i *)(drow3 + j), out);

            t1 = _mm_srli_epi16 (s1, 8);
            t2 = _mm_srli_epi16 (s2, 8);

            out = _mm_packus_epi16 (t1, t2);
            _mm_store_si128 ((__m128i *)(drow4 + j), out);
        }
    }
    if (CAM_IS_ALIGNED32 (sstride))
        return 0;

    /* Fix up the last column if necessary */
    const uint8_t * scol1 = src + 2*width;
    const uint8_t * scol2 = src + 2*width + sstride;
    uint8_t * dcol1 = dst[0] + width;
    uint8_t * dcol2 = dst[1] + width;
    uint8_t * dcol3 = dst[2] + width;
    uint8_t * dcol4 = dst[3] + width;
    __m128i t2 = _mm_set1_epi16 (0);
    for (i = 0; i < height; i++) {
        __m128i s1, t1, out;
        s1 = _mm_load_si128 ((__m128i *)(scol1 + 2*i*sstride));
        t1 = _mm_and_si128 (s1, mask);

        out = _mm_packus_epi16 (t1, t2);
        _mm_store_si128 ((__m128i *)(dcol1 + i*dstride), out);

        t1 = _mm_srli_epi16 (s1, 8);

        out = _mm_packus_epi16 (t1, t2);
        _mm_store_si128 ((__m128i *)(dcol2 + i*dstride), out);

        s1 = _mm_load_si128 ((__m128i *)(scol2 + 2*i*sstride));
        t1 = _mm_and_si128 (s1, mask);

        out = _mm_packus_epi16 (t1, t2);
        _mm_store_si128 ((__m128i *)(dcol3 + i*dstride), out);

        t1 = _mm_srli_epi16 (s1, 8);

        out = _mm_packus_epi16 (t1, t2);
        _mm_store_si128 ((__m128i *)(dcol4 + i*dstride), out);
    }
    return 0;
}

/* BOX_FILT evaluates this kernel:
 *     1  1
 *     1  1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  If stride is positive, the origin of the kernel is in the top
 * row, if negative, the origin is in the bottom row.  off is 1 to put
 * the origin in the left column of the kernel, or -1 to put the origin
 * in the right column.
 */
#define BOX_FILT(v1,v2,ptr,str,off) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) + off)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) + (str) + off)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
} while (0)

/* CROSS_FILT_VERT evaluates this kernel:
 *         1/2
 *     -1   5  -1
 *         1/2
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  The origin of the kernel is at the center.
 */
#define CROSS_FILT_VERT(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3, c10; \
    c10 = _mm_set1_epi16 (10); \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_mullo_epi16 (v1, c10); \
    v2 = _mm_mullo_epi16 (v2, c10); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) - (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    v1 = _mm_srli_epi16 (v1, 1); \
    v2 = _mm_srli_epi16 (v2, 1); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) - 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) + 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
} while (0)

/* HORIZ2_FILT evaluates this kernel:
 *     1  1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes (unused).  off is 1 to put the origin in the left column of the
 * kernel, or -1 to put the origin in the right column.
 */
#define HORIZ2_FILT(v1,v2,ptr,str,off) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) + off)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
} while (0)

/* VERT2_FILT evaluates this kernel:
 *     1
 *     1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  If stride is positive, the origin of the kernel is in the top
 * row, if negative, the origin is in the bottom row.
 */
#define VERT2_FILT(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
} while (0)

/* CROSS_FILT_SYM evaluates this kernel:
 *         -1
 *     -1   4  -1
 *         -1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  The origin of the kernel is at the center.
 */
#define CROSS_FILT_SYM(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3; \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_slli_epi16 (v1, 2); \
    v2 = _mm_slli_epi16 (v2, 2); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) - (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) - 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) + 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
} while (0)

/* CROSS_FILT_HORIZ evaluates this kernel:
 *         -1
 *     1/2  5  1/2
 *         -1
 * For a 1x16 strip of pixels of an 8u image.  v1 and v2 hold the result of
 * the computation (stored as 16s).  ptr points to the first pixel of the
 * strip, and must be 16-byte aligned.  str is the stride of image rows in
 * bytes.  The origin of the kernel is at the center.
 */
#define CROSS_FILT_HORIZ(v1,v2,ptr,str) do { \
    __m128i t1, t2, t3, c10; \
    c10 = _mm_set1_epi16 (10); \
    t1 = _mm_load_si128 ((__m128i *)(ptr)); \
    v1 = _mm_unpacklo_epi8 (t1, z); \
    v2 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_mullo_epi16 (v1, c10); \
    v2 = _mm_mullo_epi16 (v2, c10); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) - 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    t1 = _mm_loadu_si128 ((__m128i *)((ptr) + 1)); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_add_epi16 (v1, t2); \
    v2 = _mm_add_epi16 (v2, t3); \
    v1 = _mm_srli_epi16 (v1, 1); \
    v2 = _mm_srli_epi16 (v2, 1); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) - (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
    t1 = _mm_load_si128 ((__m128i *)((ptr) + (str))); \
    t2 = _mm_unpacklo_epi8 (t1, z); \
    t3 = _mm_unpackhi_epi8 (t1, z); \
    v1 = _mm_subs_epi16 (v1, t2); \
    v2 = _mm_subs_epi16 (v2, t3); \
} while (0)

#define INTERPOLATE_GB_ROW(kstride, off) do { \
    CROSS_FILT_VERT (v1, v2, gb_plane + j*sstride, kstride); \
    HORIZ2_FILT (w1, w2, b_plane + j*sstride, kstride, -off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gr_plane + j*sstride, -kstride, -off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    bg = _mm_packus_epi16 (v1, v2); \
    \
    VERT2_FILT (v1, v2, gr_plane + j*sstride, -kstride); \
    HORIZ2_FILT (w1, w2, gb_plane + j*sstride, kstride, off); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_slli_epi16 (v1, 1); \
    v2 = _mm_slli_epi16 (v2, 1); \
    CROSS_FILT_SYM (w1, w2, b_plane + j*sstride, kstride); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    gb = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_HORIZ (v1, v2, gb_plane + j*sstride, kstride); \
    VERT2_FILT (w1, w2, r_plane + j*sstride, -kstride); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gr_plane + j*sstride, -kstride, -off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    rg = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_SYM (v1, v2, b_plane + j*sstride, kstride); \
    v1 = _mm_mullo_epi16 (v1, c3); \
    v2 = _mm_mullo_epi16 (v2, c3); \
    BOX_FILT (w1, w2, r_plane + j*sstride, -kstride, off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 4); \
    v2 = _mm_srai_epi16 (v2, 4); \
    rb = _mm_packus_epi16 (v1, v2); \
    \
    gg = _mm_load_si128 ((__m128i *)(gb_plane + j*sstride)); \
    bgl1 = _mm_unpacklo_epi8 (bg, gg); \
    bgl2 = _mm_unpackhi_epi8 (bg, gg); \
    \
    a = _mm_set1_epi8 (0xff); \
    ral1 = _mm_unpacklo_epi8 (rg, a); \
    ral2 = _mm_unpackhi_epi8 (rg, a); \
    \
    bb = _mm_load_si128 ((__m128i *)(b_plane + j*sstride)); \
    bgr1 = _mm_unpacklo_epi8 (bb, gb); \
    bgr2 = _mm_unpackhi_epi8 (bb, gb); \
    \
    rar1 = _mm_unpacklo_epi8 (rb, a); \
    rar2 = _mm_unpackhi_epi8 (rb, a); \
    \
    bgral1 = _mm_unpacklo_epi16 (bgl1, ral1); \
    bgral2 = _mm_unpackhi_epi16 (bgl1, ral1); \
    bgral3 = _mm_unpacklo_epi16 (bgl2, ral2); \
    bgral4 = _mm_unpackhi_epi16 (bgl2, ral2); \
    \
    bgrar1 = _mm_unpacklo_epi16 (bgr1, rar1); \
    bgrar2 = _mm_unpackhi_epi16 (bgr1, rar1); \
    bgrar3 = _mm_unpacklo_epi16 (bgr2, rar2); \
    bgrar4 = _mm_unpackhi_epi16 (bgr2, rar2); \
} while (0)

#define INTERPOLATE_RG_ROW(kstride,off) do { \
    CROSS_FILT_SYM (v1, v2, r_plane + j*sstride, kstride); \
    v1 = _mm_mullo_epi16 (v1, c3); \
    v2 = _mm_mullo_epi16 (v2, c3); \
    BOX_FILT (w1, w2, b_plane + j*sstride, kstride, -off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 4); \
    v2 = _mm_srai_epi16 (v2, 4); \
    br = _mm_packus_epi16 (v1, v2); \
    \
    VERT2_FILT (v1, v2, gb_plane + j*sstride, kstride); \
    HORIZ2_FILT (w1, w2, gr_plane + j*sstride, kstride, -off); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_slli_epi16 (v1, 1); \
    v2 = _mm_slli_epi16 (v2, 1); \
    CROSS_FILT_SYM (w1, w2, r_plane + j*sstride, kstride); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    gr = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_HORIZ (v1, v2, gr_plane + j*sstride, kstride); \
    VERT2_FILT (w1, w2, b_plane + j*sstride, kstride); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gb_plane + j*sstride, kstride, off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    bg = _mm_packus_epi16 (v1, v2); \
    \
    CROSS_FILT_VERT (v1, v2, gr_plane + j*sstride, kstride); \
    HORIZ2_FILT (w1, w2, r_plane + j*sstride, kstride, off); \
    w1 = _mm_slli_epi16 (w1, 2); \
    w2 = _mm_slli_epi16 (w2, 2); \
    v1 = _mm_add_epi16 (v1, w1); \
    v2 = _mm_add_epi16 (v2, w2); \
    BOX_FILT (w1, w2, gb_plane + j*sstride, kstride, off); \
    v1 = _mm_subs_epi16 (v1, w1); \
    v2 = _mm_subs_epi16 (v2, w2); \
    v1 = _mm_srai_epi16 (v1, 3); \
    v2 = _mm_srai_epi16 (v2, 3); \
    rg = _mm_packus_epi16 (v1, v2); \
    \
    bgl1 = _mm_unpacklo_epi8 (br, gr); \
    bgl2 = _mm_unpackhi_epi8 (br, gr); \
    \
    rr = _mm_load_si128 ((__m128i *)(r_plane + j*sstride)); \
    a = _mm_set1_epi8 (0xff); \
    ral1 = _mm_unpacklo_epi8 (rr, a); \
    ral2 = _mm_unpackhi_epi8 (rr, a); \
    \
    gg = _mm_load_si128 ((__m128i *)(gr_plane + j*sstride)); \
    bgr1 = _mm_unpacklo_epi8 (bg, gg); \
    bgr2 = _mm_unpackhi_epi8 (bg, gg); \
    \
    rar1 = _mm_unpacklo_epi8 (rg, a); \
    rar2 = _mm_unpackhi_epi8 (rg, a); \
    \
    bgral1 = _mm_unpacklo_epi16 (bgl1, ral1); \
    bgral2 = _mm_unpackhi_epi16 (bgl1, ral1); \
    bgral3 = _mm_unpacklo_epi16 (bgl2, ral2); \
    bgral4 = _mm_unpackhi_epi16 (bgl2, ral2); \
    \
    bgrar1 = _mm_unpacklo_epi16 (bgr1, rar1); \
    bgrar2 = _mm_unpackhi_epi16 (bgr1, rar1); \
    bgrar3 = _mm_unpacklo_epi16 (bgr2, rar2); \
    bgrar4 = _mm_unpackhi_epi16 (bgr2, rar2); \
} while (0)

int
cam_pixel_bayer_interpolate_to_8u_bgra_sse2 (uint8_t ** src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format)
{
    int i, j;
    for (i = 0; i < 4; i++) {
        if (!CAM_IS_ALIGNED16(src[i]) || !CAM_IS_ALIGNED16(sstride)) {
            fprintf (stderr, "%s: src[%d] is not 16-byte aligned\n",
                    __FUNCTION__, i);
            return -1;
        }
    }
    if (!CAM_IS_ALIGNED16(dst) || !CAM_IS_ALIGNED128(dstride)) {
        fprintf (stderr, "%s: dst is not 16-byte aligned or 128-byte stride "
                "aligned\n", __FUNCTION__);
        return -1;
    }

    __m128i z = _mm_set1_epi32 (0);
    __m128i c3 = _mm_set1_epi16 (3);
    __m128i bg, gb, rg, rb, gg, a, bb, br, gr, rr;
    __m128i bgl1, bgl2, ral1, ral2;
    __m128i bgr1, bgr2, rar1, rar2;
    __m128i bgral1, bgral2, bgral3, bgral4;
    __m128i bgrar1, bgrar2, bgrar3, bgrar4;
    __m128i v1, v2, w1, w2;

    if (format == CAM_PIXEL_FORMAT_BAYER_GBRG ||
            format == CAM_PIXEL_FORMAT_BAYER_RGGB) {
        int drow_offset1 = 0;
        int drow_offset2 = dstride;
        int kernel_stride = sstride;
        uint8_t * gb_plane = src[0];
        uint8_t * b_plane = src[1];
        uint8_t * r_plane = src[2];
        uint8_t * gr_plane = src[3];
        if (format == CAM_PIXEL_FORMAT_BAYER_RGGB) {
            drow_offset1 = dstride;
            drow_offset2 = 0;
            kernel_stride = -sstride;
            r_plane = src[0];
            gr_plane = src[1];
            gb_plane = src[2];
            b_plane = src[3];
        }

        for (i = 0; i < width/2; i += 16) {
            uint8_t * dcol = dst + i*8;

            for (j = 0; j < height/2; j++) {
                INTERPOLATE_GB_ROW (kernel_stride, 1);

                uint8_t * drow = dcol + j*2*dstride + drow_offset1;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgral4, bgrar4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgral4, bgrar4));

                INTERPOLATE_RG_ROW (kernel_stride, 1);

                drow = dcol + j*2*dstride + drow_offset2;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgral1, bgrar1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgral2, bgrar2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgral3, bgrar3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgral4, bgrar4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgral4, bgrar4));

            }
            gb_plane += 16;
            b_plane += 16;
            r_plane += 16;
            gr_plane += 16;
        }
    }
    else {
        int drow_offset1 = 0;
        int drow_offset2 = dstride;
        int kernel_stride = sstride;
        uint8_t * b_plane = src[0];
        uint8_t * gb_plane = src[1];
        uint8_t * gr_plane = src[2];
        uint8_t * r_plane = src[3];
        if (format == CAM_PIXEL_FORMAT_BAYER_GRBG) {
            drow_offset1 = dstride;
            drow_offset2 = 0;
            kernel_stride = -sstride;
            gr_plane = src[0];
            r_plane = src[1];
            b_plane = src[2];
            gb_plane = src[3];
        }

        for (i = 0; i < width/2; i += 16) {
            uint8_t * dcol = dst + i*8;

            for (j = 0; j < height/2; j++) {
                INTERPOLATE_GB_ROW (kernel_stride, -1);

                uint8_t * drow = dcol + j*2*dstride + drow_offset1;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgrar4, bgral4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgrar4, bgral4));

                INTERPOLATE_RG_ROW (kernel_stride, -1);

                drow = dcol + j*2*dstride + drow_offset2;
                _mm_store_si128 ((__m128i *)drow,
                        _mm_unpacklo_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+16),
                        _mm_unpackhi_epi32 (bgrar1, bgral1));
                _mm_store_si128 ((__m128i *)(drow+32),
                        _mm_unpacklo_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+48),
                        _mm_unpackhi_epi32 (bgrar2, bgral2));
                _mm_store_si128 ((__m128i *)(drow+64),
                        _mm_unpacklo_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+80),
                        _mm_unpackhi_epi32 (bgrar3, bgral3));
                _mm_store_si128 ((__m128i *)(drow+96),
                        _mm_unpacklo_epi32 (bgrar4, bgral4));
                _mm_store_si128 ((__m128i *)(drow+112),
                        _mm_unpackhi_epi32 (bgrar4, bgral4));

            }
            gb_plane += 16;
            b_plane += 16;
            r_plane += 16;
            gr_plane += 16;
        }
    }
    return 0;
}

#define INTERPOLATE_GRAY_ROW_GX() do { \
    v = _mm_load_si128 ((__m128i *)(srow + j)); \
    v1l = _mm_and_si128 (v, mask); \
    v1r = _mm_srli_epi16 (v, 8); \
    \
    tl = _mm_subs_epi16 (z, v1l); \
    _mm_store_si128 (tmp + i1 + j/8, tl); \
    \
    v = _mm_mullo_epi16 (v1r, c7); \
    tr = _mm_subs_epi16 (z, v); \
    _mm_store_si128 (tmp + i1 + j/8 + 1, tr); \
    \
    tl = _mm_load_si128 (tmp + i5 + j/8); \
    tl = _mm_subs_epi16 (tl, v1l); \
    tl = _mm_srai_epi16 (tl, 6); \
    tl = _mm_packus_epi16 (tl, tl); \
    \
    tr = _mm_load_si128 (tmp + i5 + j/8 + 1); \
    tr = _mm_subs_epi16 (tr, v); \
    tr = _mm_srai_epi16 (tr, 6); \
    tr = _mm_packus_epi16 (tr, tr); \
    \
    v = _mm_unpacklo_epi8 (tl, tr); \
    _mm_store_si128 ((__m128i *)(drow + j), v); \
    \
    tl1 = _mm_load_si128 (tmp + i2 + j/8); \
    tl2 = _mm_load_si128 (tmp + i4 + j/8); \
    v = _mm_slli_epi16 (v1r, 2); \
    tl1 = _mm_add_epi16 (tl1, v); \
    tl2 = _mm_add_epi16 (tl2, v); \
    \
    v = _mm_loadu_si128 ((__m128i *)(srow + j - 2)); \
    v2l = _mm_and_si128 (v, mask); \
    v2r = _mm_srli_epi16 (v, 8); \
    \
    v = _mm_slli_epi16 (v2r, 2); \
    tl1 = _mm_add_epi16 (tl1, v); \
    tl2 = _mm_add_epi16 (tl2, v); \
    \
    v = _mm_slli_epi16 (v1l, 3); \
    tl1 = _mm_add_epi16 (tl1, v); \
    tl2 = _mm_add_epi16 (tl2, v); \
    _mm_store_si128 (tmp + i2 + j/8, tl1); \
    _mm_store_si128 (tmp + i4 + j/8, tl2); \
    \
    tr1 = _mm_load_si128 (tmp + i2 + j/8 + 1); \
    tr2 = _mm_load_si128 (tmp + i4 + j/8 + 1); \
    v = _mm_slli_epi16 (v1r, 3); \
    tr1 = _mm_add_epi16 (tr1, v); \
    tr2 = _mm_add_epi16 (tr2, v); \
    \
    v = _mm_slli_epi16 (v1l, 2); \
    tr1 = _mm_subs_epi16 (tr1, v); \
    tr2 = _mm_subs_epi16 (tr2, v); \
    \
    v = _mm_loadu_si128 ((__m128i *)(srow + j + 2)); \
    v3l = _mm_and_si128 (v, mask); \
    v3r = _mm_srli_epi16 (v, 8); \
    \
    v = _mm_slli_epi16 (v3l, 2); \
    tr1 = _mm_subs_epi16 (tr1, v); \
    tr2 = _mm_subs_epi16 (tr2, v); \
    \
    _mm_store_si128 (tmp + i2 + j/8 + 1, tr1); \
    _mm_store_si128 (tmp + i4 + j/8 + 1, tr2); \
    \
    tl1 = _mm_load_si128 (tmp + i3 + j/8); \
    v = _mm_mullo_epi16 (v1l, c52); \
    tl1 = _mm_add_epi16 (tl1, v); \
    v = _mm_slli_epi16 (v2r, 3); \
    tl1 = _mm_add_epi16 (tl1, v); \
    v = _mm_slli_epi16 (v1r, 3); \
    tl1 = _mm_add_epi16 (tl1, v); \
    tl1 = _mm_subs_epi16 (tl1, v2l); \
    tl1 = _mm_subs_epi16 (tl1, v3l); \
    _mm_store_si128 (tmp + i3 + j/8, tl1); \
    \
    tr1 = _mm_load_si128 (tmp + i3 + j/8 + 1); \
    v = _mm_mullo_epi16 (v1r, c44); \
    tr1 = _mm_add_epi16 (tr1, v); \
    v = _mm_slli_epi16 (v1l, 3); \
    tr1 = _mm_add_epi16 (tr1, v); \
    v = _mm_slli_epi16 (v3l, 3); \
    tr1 = _mm_add_epi16 (tr1, v); \
    v = _mm_mullo_epi16 (v2r, c7); \
    tr1 = _mm_subs_epi16 (tr1, v); \
    v = _mm_mullo_epi16 (v3r, c7); \
    tr1 = _mm_subs_epi16 (tr1, v); \
    _mm_store_si128 (tmp + i3 + j/8 + 1, tr1); \
} while (0)

#define INTERPOLATE_GRAY_ROW_XG() do { \
    v = _mm_load_si128 ((__m128i *)(srow + j)); \
    v1l = _mm_and_si128 (v, mask); \
    v1r = _mm_srli_epi16 (v, 8); \
    \
    v = _mm_mullo_epi16 (v1l, c7); \
    tl = _mm_subs_epi16 (z, v); \
    _mm_store_si128 (tmp + i1 + j/8, tl); \
    \
    tr = _mm_subs_epi16 (z, v1r); \
    _mm_store_si128 (tmp + i1 + j/8 + 1, tr); \
    \
    tl = _mm_load_si128 (tmp + i5 + j/8); \
    tl = _mm_subs_epi16 (tl, v); \
    tl = _mm_srai_epi16 (tl, 6); \
    tl = _mm_packus_epi16 (tl, tl); \
    \
    tr = _mm_load_si128 (tmp + i5 + j/8 + 1); \
    tr = _mm_subs_epi16 (tr, v1r); \
    tr = _mm_srai_epi16 (tr, 6); \
    tr = _mm_packus_epi16 (tr, tr); \
    \
    v = _mm_unpacklo_epi8 (tl, tr); \
    _mm_store_si128 ((__m128i *)(drow + j), v); \
    \
    tl1 = _mm_load_si128 (tmp + i2 + j/8); \
    tl2 = _mm_load_si128 (tmp + i4 + j/8); \
    v = _mm_slli_epi16 (v1r, 2); \
    tl1 = _mm_subs_epi16 (tl1, v); \
    tl2 = _mm_subs_epi16 (tl2, v); \
    \
    v = _mm_loadu_si128 ((__m128i *)(srow + j - 2)); \
    v2l = _mm_and_si128 (v, mask); \
    v2r = _mm_srli_epi16 (v, 8); \
    \
    v = _mm_slli_epi16 (v2r, 2); \
    tl1 = _mm_subs_epi16 (tl1, v); \
    tl2 = _mm_subs_epi16 (tl2, v); \
    \
    v = _mm_slli_epi16 (v1l, 3); \
    tl1 = _mm_add_epi16 (tl1, v); \
    tl2 = _mm_add_epi16 (tl2, v); \
    _mm_store_si128 (tmp + i2 + j/8, tl1); \
    _mm_store_si128 (tmp + i4 + j/8, tl2); \
    \
    tr1 = _mm_load_si128 (tmp + i2 + j/8 + 1); \
    tr2 = _mm_load_si128 (tmp + i4 + j/8 + 1); \
    v = _mm_slli_epi16 (v1r, 3); \
    tr1 = _mm_add_epi16 (tr1, v); \
    tr2 = _mm_add_epi16 (tr2, v); \
    \
    v = _mm_slli_epi16 (v1l, 2); \
    tr1 = _mm_add_epi16 (tr1, v); \
    tr2 = _mm_add_epi16 (tr2, v); \
    \
    v = _mm_loadu_si128 ((__m128i *)(srow + j + 2)); \
    v3l = _mm_and_si128 (v, mask); \
    v3r = _mm_srli_epi16 (v, 8); \
    \
    v = _mm_slli_epi16 (v3l, 2); \
    tr1 = _mm_add_epi16 (tr1, v); \
    tr2 = _mm_add_epi16 (tr2, v); \
    \
    _mm_store_si128 (tmp + i2 + j/8 + 1, tr1); \
    _mm_store_si128 (tmp + i4 + j/8 + 1, tr2); \
    \
    tl1 = _mm_load_si128 (tmp + i3 + j/8); \
    v = _mm_mullo_epi16 (v1l, c44); \
    tl1 = _mm_add_epi16 (tl1, v); \
    v = _mm_slli_epi16 (v2r, 3); \
    tl1 = _mm_add_epi16 (tl1, v); \
    v = _mm_slli_epi16 (v1r, 3); \
    tl1 = _mm_add_epi16 (tl1, v); \
    v = _mm_mullo_epi16 (v2l, c7); \
    tl1 = _mm_subs_epi16 (tl1, v); \
    v = _mm_mullo_epi16 (v3l, c7); \
    tl1 = _mm_subs_epi16 (tl1, v); \
    _mm_store_si128 (tmp + i3 + j/8, tl1); \
    \
    tr1 = _mm_load_si128 (tmp + i3 + j/8 + 1); \
    v = _mm_mullo_epi16 (v1r, c52); \
    tr1 = _mm_add_epi16 (tr1, v); \
    v = _mm_slli_epi16 (v1l, 3); \
    tr1 = _mm_add_epi16 (tr1, v); \
    v = _mm_slli_epi16 (v3l, 3); \
    tr1 = _mm_add_epi16 (tr1, v); \
    tr1 = _mm_subs_epi16 (tr1, v2r); \
    tr1 = _mm_subs_epi16 (tr1, v3r); \
    _mm_store_si128 (tmp + i3 + j/8 + 1, tr1); \
} while (0)

int
cam_pixel_bayer_interpolate_to_8u_gray_sse2 (uint8_t * src, int sstride,
        uint8_t * dst, int dstride, int width, int height,
        CamPixelFormat format)
{
    int i, j;
    if (!CAM_IS_ALIGNED16(src) || !CAM_IS_ALIGNED16(sstride)) {
        fprintf (stderr, "%s: src is not 16-byte aligned\n",
                __FUNCTION__);
        return -1;
    }
    if (!CAM_IS_ALIGNED16(dst) || !CAM_IS_ALIGNED16(dstride)) {
        fprintf (stderr, "%s: dst is not 16-byte aligned\n",
                __FUNCTION__);
        return -1;
    }

    int tmpstride = (width + 15) / 16 * 2;
    __m128i tmp[tmpstride * 5];
    __m128i mask = _mm_set1_epi16 (0xff);
    __m128i z = _mm_set1_epi16 (0);
    __m128i c7 = _mm_set1_epi16 (7);
    __m128i c44 = _mm_set1_epi16 (44);
    __m128i c52 = _mm_set1_epi16 (52);

    __m128i v, v1l, v1r, v2l, v2r, v3l, v3r;
    __m128i tl, tr, tl1, tl2, tr1, tr2;

    if (format == CAM_PIXEL_FORMAT_BAYER_GBRG ||
            format == CAM_PIXEL_FORMAT_BAYER_GRBG) {
        for (i = -2; i < height + 2; i += 2) {
            uint8_t * drow = dst + (i>=2)*(i-2)*dstride;
            uint8_t * srow = src + i*sstride;
            int i5 = tmpstride * (i % 5);
            int i4 = tmpstride * ((i+1) % 5);
            int i3 = tmpstride * ((i+2) % 5);
            int i2 = tmpstride * ((i+3) % 5);
            int i1 = tmpstride * ((i+4) % 5);

            for (j = 0; j < width; j += 16)
                INTERPOLATE_GRAY_ROW_GX();

            drow += dstride;
            srow += sstride;
            i5 = tmpstride * ((i+1) % 5);
            i4 = tmpstride * ((i+2) % 5);
            i3 = tmpstride * ((i+3) % 5);
            i2 = tmpstride * ((i+4) % 5);
            i1 = tmpstride * ((i+5) % 5);

            for (j = 0; j < width; j += 16)
                INTERPOLATE_GRAY_ROW_XG();
        }
    }
    else {
        for (i = -2; i < height + 2; i += 2) {
            uint8_t * drow = dst + (i>=2)*(i-2)*dstride;
            uint8_t * srow = src + i*sstride;
            int i5 = tmpstride * (i % 5);
            int i4 = tmpstride * ((i+1) % 5);
            int i3 = tmpstride * ((i+2) % 5);
            int i2 = tmpstride * ((i+3) % 5);
            int i1 = tmpstride * ((i+4) % 5);

            for (j = 0; j < width; j += 16)
                INTERPOLATE_GRAY_ROW_XG();

            drow += dstride;
            srow += sstride;
            i5 = tmpstride * ((i+1) % 5);
            i4 = tmpstride * ((i+2) % 5);
            i3 = tmpstride * ((i+3) % 5);
            i2 = tmpstride * ((i+4) % 5);
            i1 = tmpstride * ((i+5) % 5);

            for (j = 0; j < width; j += 16)
                INTERPOLATE_GRAY_ROW_GX();
        }
    }
    return 0;
}

