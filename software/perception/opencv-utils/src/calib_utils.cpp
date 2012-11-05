#include <stdio.h>
#include "opencv_utils.hpp"

// using namespace opencv_utils;
using namespace cv;

// normalize the point
inline Point2f normalize_point(const Point2f& p, Mat_<double>& Kmat) { 
    double* K = (double*)Kmat.data;
    
    Point2f np;
    np.x = (p.x - K[2]) / K[0];
    np.y = (p.y - K[5]) / K[4];

    return np;
}

// backproject to 3D with a given depth
Point3f backproject(const Point2f& p, double depth, Mat_<double>& Kmat, Mat_<double>& Rmat, Mat_<double>& tmat) { 

    Point2f np = normalize_point(p, Kmat);
    
    double* K = (double*)Kmat.data;
    double* R = (double*)Rmat.data;
    double* t = (double*)tmat.data;


    double t2 = depth * np.x - t[0];
    double t5 = depth * np.y - t[1];
    double t7 = depth - t[2];

    Point3f p3; 
    p3.x = R[0] * t2 + R[3] * t5 + R[6] * t7;
    p3.y = R[1] * t2 + R[4] * t5 + R[7] * t7;
    p3.z = R[2] * t2 + R[5] * t5 + R[8] * t7;

    return p3;
}


// conversion from double
Mat_<double> distortion_to_mat(double* d) { 
    Mat_<double> D(5,1,d);    
    return D;
}

// Extracts the extrinsics in T = [rvec tvec] where rvec is the rodrigues form, and tvec is as usual
void extract_camera_extrinsics(Mat_<double>& T, cv::Mat_<double>& rvec, cv::Mat_<double>& tvec) { 
    tvec = T(Rect(3,0,1,3));
    Rodrigues(T(Rect(0,0,3,3)), rvec);
    return;
}

// Combines R and t to form T = [R t]
Mat_<double> Rt_combine(Mat_<double>& R, Mat_<double>& t) { 
    Mat_<double> T = Mat_<double>::eye(4,4);
    Mat R_roi(T, Rect(0,0,3,3));
    R.copyTo(R_roi);
    Mat t_roi(T, Rect(3,0,1,3));
    t.copyTo(t_roi);
    return T;
}

// Extracts R and t from T = [R t]
void Rt_extract(Mat_<double>& T, Mat_<double>& R, Mat_<double>& t) { 
    t = T(Rect(3,0,1,3)).clone();
    R = T(Rect(0,0,3,3)).clone();
    return;
}

// Assumption is that this is T = [R t] where R is a rotational transform (orthogonal), and t is translation
Mat_<double> invert_T(Mat_<double>& T) { 
    // Rinv = R', tinv = - R' * t
    Mat_<double> Rt(3,3);
    transpose(T(Rect(0,0,3,3)),Rt);
    Mat_<double> tinv = - Rt * T(Rect(3,0,1,3));
    
    Mat_<double> Tinv = Rt_combine(Rt, tinv);
    return Tinv;
}

// pinhole camera = [fx fy skew cx cy] -> K
cv::Mat_<double> pinhole_to_mat(double* pinhole) { 

    cv::Mat_<double> K = cv::Mat_<double>::eye(3,3);
    K(0, 0) = pinhole[0]; // fx 
    K(1, 1) = pinhole[1]; // fy

    K(0, 1) = pinhole[2]; // skew

    K(0, 2) = pinhole[3]; // cx
    K(1, 2) = pinhole[4]; // cy

    K(2, 2) = 1;
    return K;
}
