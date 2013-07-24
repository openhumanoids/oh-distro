
/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Stuart Glaser
 */

#ifndef EIGEN_KDL_CONVERSIONS_HPP
#define EIGEN_KDL_CONVERSIONS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/frames.hpp>
#include <lcmtypes/drc_lcmtypes.hpp>

namespace visualization_utils {



static inline void rotate_eigen_vector_given_kdl_frame(const Eigen::Vector3d &in,const KDL::Frame &T_out_in,Eigen::Vector3d &out)
{
   KDL::Vector temp;
   temp[0] =  in[0]; temp[1] =  in[1]; temp[2] = in[2];
   temp =  T_out_in*temp;
//     Eigen::Vector3d eigen_temp(temp.data);
//     out=eigen_temp;
   out[0] =  temp[0]; out[1] =  temp[1]; out[2] = temp[2];
}

static inline void rotate_eigen_vector_given_kdl_frame(const Eigen::Vector3f &in,const KDL::Frame &T_out_in,Eigen::Vector3d &out)
{
   KDL::Vector temp;
   temp[0] =  in[0]; temp[1] =  in[1]; temp[2] = in[2];
   temp =  T_out_in*temp;
   out[0] =  temp[0]; out[1] =  temp[1]; out[2] = temp[2];
}

static inline void rotate_eigen_vector_given_kdl_frame(const Eigen::Vector3f &in,const KDL::Frame &T_out_in,Eigen::Vector3f &out)
{
   KDL::Vector temp;
   temp[0] =  in[0]; temp[1] =  in[1]; temp[2] = in[2];
   temp =  T_out_in*temp;
   out[0] =  (float) temp[0]; out[1] =  (float) temp[1]; out[2] = (float) temp[2];
}
  
/// Converts a KDL twist into an Eigen matrix
static inline void transformKDLtwistToEigen(const KDL::Twist &k, Eigen::Matrix<double, 6, 1> &e)
{
  e[0] = k.vel.x();
  e[1] = k.vel.y();
  e[2] = k.vel.z();
  e[3] = k.rot.x();
  e[4] = k.rot.y();
  e[5] = k.rot.z();
};

/// Converts a LCM twist into an Eigen matrix
static inline void transformLCMtwistToEigen(const drc::twist_t &t, Eigen::Matrix<double, 6, 1> &e)
{

KDL::Vector lin_vel(t.linear_velocity.x,t.linear_velocity.y,t.linear_velocity.z);
KDL::Vector ang_vel(t.angular_velocity.x,t.angular_velocity.y,t.angular_velocity.z);
KDL::Twist k(lin_vel,ang_vel);
transformKDLtwistToEigen(k,e);
};

/// Converts a KDL frame into an Eigen transform
static inline void transformKDLToEigen(const KDL::Frame &k, Eigen::Affine3d &e)
{
  e(0,3) = k.p[0];
  e(1,3) = k.p[1];
  e(2,3) = k.p[2];

  e(0,0) = k.M(0,0);
  e(0,1) = k.M(0,1);
  e(0,2) = k.M(0,2);
  e(1,0) = k.M(1,0);
  e(1,1) = k.M(1,1);
  e(1,2) = k.M(1,2);
  e(2,0) = k.M(2,0);
  e(2,1) = k.M(2,1);
  e(2,2) = k.M(2,2);

  e(3,0) = 0.0;
  e(3,1) = 0.0;
  e(3,2) = 0.0;
  e(3,3) = 1.0;
};


/// Converts a LCM position3D_t into an Eigen matrix
static inline void transformLCMToEigen(const drc::position_3d_t &t, Eigen::Affine3d &e)
{
   //TransformLCMToKDLFrame
    KDL::Frame k;
    k.p[0] =t.translation.x;
    k.p[1] =t.translation.y;
    k.p[2] =t.translation.z;
    //  Eigen::Quaterniond q(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
    // q.normalize();
    KDL::Rotation M;
    //M =  KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
    M =  KDL::Rotation::Quaternion(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
    k.M = M;
    transformKDLToEigen(k, e);
};


static inline void transformLCMToKDL(const drc::position_3d_t &t,  KDL::Frame &k)
{
   //TransformLCMToKDLFrame
    k.p[0] =t.translation.x;
    k.p[1] =t.translation.y;
    k.p[2] =t.translation.z;
    //  Eigen::Quaterniond q(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
    // q.normalize();
    KDL::Rotation M;
    //M =  KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w());
    M =  KDL::Rotation::Quaternion(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w);
    k.M = M;
};


static inline void transformKDLToLCM(const KDL::Frame &k,drc::position_3d_t &t)
{
   //TransformLCMToKDLFrame
    t.translation.x=k.p[0];
    t.translation.y=k.p[1];
    t.translation.z=k.p[2];
    //  Eigen::Quaterniond q(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
    // q.normalize();
    double x,y,z,w;
    k.M.GetQuaternion(x,y,z,w);
    t.rotation.x=x;
    t.rotation.y=y;
    t.rotation.z=z;
    t.rotation.w=w;
    
};


/// Converts a KDL frame into an Eigen transform
static inline void transformEigenToKDL(const Eigen::Affine3d &e, KDL::Frame &k)
{
  k.p[0]=  e(0,3);
  k.p[1]=  e(1,3);
  k.p[2]=  e(2,3);

  k.M(0,0) = e(0,0) ;
  k.M(0,1) = e(0,1) ;
  k.M(0,2) = e(0,2) ;
  k.M(1,0) = e(1,0) ;
  k.M(1,1) = e(1,1);
  k.M(1,2) = e(1,2);
  k.M(2,0) = e(2,0);
  k.M(2,1) = e(2,1);
  k.M(2,2) = e(2,2);

};


static inline void transformEigenToLCM(const Eigen::Affine3d &e, drc::position_3d_t &m)
{
  m.translation.x = e.translation()[0];
  m.translation.y = e.translation()[1];
  m.translation.z = e.translation()[2];
  Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
  q.normalize();
  m.rotation.x = q.x();
  m.rotation.y = q.y();
  m.rotation.z = q.z();
  m.rotation.w = q.w();
  if (m.rotation.w < 0) {
    m.rotation.x *= -1;
    m.rotation.y *= -1;
    m.rotation.z *= -1;
    m.rotation.w *= -1;
  }
};


} // namespace

#endif
