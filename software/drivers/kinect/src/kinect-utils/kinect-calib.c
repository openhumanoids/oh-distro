#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "kinect-calib.h"

KinectCalibration* 
kinect_calib_new(void)
{
    return (KinectCalibration*) calloc(1, sizeof(KinectCalibration));
}

void 
kinect_calib_destroy(KinectCalibration* kcal)
{
    memset(kcal, 0, sizeof(KinectCalibration));
    free(kcal);
}

void 
kinect_calib_get_depth_uvd_to_depth_xyz_4x4(const KinectCalibration* kcal, 
        double result[16])
{
#if 0
    double k_inv[] = {
        1, 0, 0, -kcal->intrinsics_depth.cx,
        0, 1, 0, -kcal->intrinsics_depth.cy,
        0, 0, 0, kcal->intrinsics_depth.fx,
        0, 0, 1/kcal->projector_depth_baseline, 0
    };
#else
    double fx_inv = 1 / kcal->intrinsics_depth.fx;
    double pdb_inv = 1 / kcal->projector_depth_baseline;
    double a = -0.125 * fx_inv * pdb_inv;
    double b = 0.125 * kcal->shift_offset * fx_inv * pdb_inv;
    double cx = kcal->intrinsics_depth.cx;
    double cy = kcal->intrinsics_depth.cy;
    double k_inv[] = {
        fx_inv, 0, 0, -cx * fx_inv,
        0, fx_inv, 0, -cy * fx_inv,
        0, 0, 0, 1,
        0, 0, a, b 
    };
#endif
    memcpy(result, k_inv, 16*sizeof(double));
}

void 
kinect_calib_get_depth_xyz_to_rgb_xyz_4x4(const KinectCalibration* kcal, 
        double result[16])
{
    const double* R = kcal->depth_to_rgb_rot;
    const double* T = kcal->depth_to_rgb_translation;
    double M[] = {
        R[0], R[1], R[2], T[0],
        R[3], R[4], R[5], T[1],
        R[6], R[7], R[8], T[2],
        0, 0, 0, 1
    };
    memcpy(result, M, 16*sizeof(double));
}

void 
kinect_calib_get_rgb_xyz_to_rgb_uvw_3x4(const KinectCalibration* kcal, 
        double result[12])
{
    double f = kcal->intrinsics_rgb.fx;
    double cx = kcal->intrinsics_rgb.cx;
    double cy = kcal->intrinsics_rgb.cy;
    double k[] = {
        f, 0, cx, 0,
        0, f, cy, 0,
        0, 0,  1, 0
    };
    memcpy(result, k, 12*sizeof(double));
}

static inline void
_matrix_multiply (const double *a, int a_nrows, int a_ncols,
        const double *b, int b_nrows, int b_ncols,
        double *result)
{
    int i, j, r;
    for (i=0; i<a_nrows; i++) {
        for (j=0; j<b_ncols; j++) {
            double acc = 0;
            for (r=0; r<a_ncols; r++) {
                acc += a[i*a_ncols + r] * b[r*b_ncols + j];
            }
            result[i*b_ncols + j] = acc;
        }
    }
}

void 
kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(const KinectCalibration* kcal, 
        double result[12])
{
    double d_uvd_to_d_xyz[16];
    kinect_calib_get_depth_uvd_to_depth_xyz_4x4(kcal, d_uvd_to_d_xyz);

    double d_xyz_to_rgb_xyz[16];
    kinect_calib_get_depth_xyz_to_rgb_xyz_4x4(kcal, d_xyz_to_rgb_xyz);

    double rgb_xyz_to_rgb_uvw[12];
    kinect_calib_get_rgb_xyz_to_rgb_uvw_3x4(kcal, rgb_xyz_to_rgb_uvw);

    double t1[16];
    _matrix_multiply(d_xyz_to_rgb_xyz, 4, 4, 
            d_uvd_to_d_xyz, 4, 4, t1);
    _matrix_multiply(rgb_xyz_to_rgb_uvw, 3, 4, 
            t1, 4, 4, 
            result);
}
