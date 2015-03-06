#ifndef __kinect_kinect_calib_h__
#define __kinect_kinect_calib_h__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _KinectCameraIntrinsics
{
    double fx; // focal length
    double cx; // center of projection x
    double cy; // center of projection y
    double k1; // distortion parameter k1
    double k2; // distortion parameter k2

    double p1; // not used
    double p2; // not used
    double k3; // not used
} KinectCameraIntrinsics;

typedef struct _KinectCalibration
{
    int width;   // image width
    int height;  // image height

    double shift_offset;                // fixed disparity offset
    double projector_depth_baseline;    // distance from projector to depth camera (m)
    double depth_to_rgb_rot[9];         // 3x3 rotation matrix
    double depth_to_rgb_translation[3]; // x, y, z

    KinectCameraIntrinsics intrinsics_depth;
    KinectCameraIntrinsics intrinsics_rgb;
} KinectCalibration;

KinectCalibration* kinect_calib_new(void);

void kinect_calib_destroy(KinectCalibration* kcal);

/**
 * Compute the 4x4 transformation matrix mapping [ u, v, disparity, 1 ]
 * coordinates to [ x, y, z, w ] homogeneous coordinates in depth camera space.
 * If u, and v are the coordinates of a depth camera pixel, and disparity is
 * the raw disparity, then [ x, y, z, w ] are computed by:
 *
 *   double uvd_depth[4] = { 
 *     u_d, 
 *     v_d, 
 *     0.125 * (kcal->shift_offset - disparity), 
 *     1 
 *   };
 *
 *   double M[16];
 *   kinect_calib_get_depth_uvd_to_depth_xyz_4x4(kcal, M);
 *
 *   double xyzw[4];
 *   // replace with your favorite matrix-vector multiplication function
 *   matrix_vector_multiply(M, uvd_depth, xyzw);  
 *   
 *   // normalize homogeneous coordinates
 *   double x = xyzw[0] / xyzw[3];
 *   double y = xyzw[1] / xyzw[3];
 *   double z = xyzw[2] / xyzw[3];
 */
void kinect_calib_get_depth_uvd_to_depth_xyz_4x4(const KinectCalibration* kcal, 
        double result[16]);

/**
 * Compute the 4x4 transformation matrix mapping [ x, y, z, w ] coordinates in 
 * depth camera space to RGB camera space.
 */
void kinect_calib_get_depth_xyz_to_rgb_xyz_4x4(const KinectCalibration* kcal, 
        double result[16]);

/**
 * Compute the 3x4 transformation matrix projecting [ x, y, z, w ] coordinates in 
 * RGB camera space to RGB image space.
 */
void kinect_calib_get_rgb_xyz_to_rgb_uvw_3x4(const KinectCalibration* kcal, 
        double result[12]);

/**
 * Computes the distorted pixel coordinate for an RGB camera pixel from a
 * rectified pixel coordinate.  This is not particularly fast..
 */
inline void kinect_calib_distort_rgb_uv(const KinectCalibration* kcal, 
        const double* uv_rect, 
        double* result)
{
    const KinectCameraIntrinsics *k = &kcal->intrinsics_rgb;
    double du = (uv_rect[0] - k->cx) / k->fx;
    double dv = (uv_rect[1] - k->cy) / k->fx;
    double rad_2 = du*du + dv*dv;
    double rad_4 = rad_2*rad_2;
    double s = (1 + k->k1 * rad_2 + k->k2 * rad_4) * k->fx;
    result[0] = du * s + k->cx;
    result[1] = dv * s + k->cy;
}

// ============= convenience functions =================

/**
 * Convenience function to retrieve the transformation mapping [u, v,
 * disparity] coordinates in the depth camera space to [u, v, w] coordinates in
 * rectified RGB image space.  In most cases, the w coordinate will not be 1,
 * so to normalize the [u, v, w] homogeneous coordinate to pixel coordinates, u
 * and v need to be divided by w.
 *
 * If u_d, v_d, and disparity are the image coordinates and raw disparity value
 * of a pixel from a depth image, then the mapping to an RGB pixel is:
 *
 *   double uvd_depth[4] = { 
 *     u_d, 
 *     v_d, 
 *     0.125 * (kcal->shift_offset - disparity), 
 *     1 
 *   };
 *
 *   double M[12];
 *   kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(kcal, M);
 *
 *   double uvw_rgb[3];
 *   // replace with your favorite matrix-vector multiplication function
 *   matrix_vector_multiply(M, uvd_depth, uvw_rgb);  
 * 
 *   // normalize homogeneous coordinates
 *   double uv[2] = {
 *     uvw_rgb[0] / uvw_rgb[2],
 *     uvw_rgb[1] / uvw_rgb[2]
 *   };
 *  
 *   // optionally, compute distorted pixel coordinates
 *   double uv_dist[2];
 *   kinect_calib_distort_rgb_uv(kcal, uv, uv_dist);
 */
void kinect_calib_get_depth_uvd_to_rgb_uvw_3x4(const KinectCalibration* kcal, 
        double result[12]);


#ifdef __cplusplus
}
#endif

#endif
