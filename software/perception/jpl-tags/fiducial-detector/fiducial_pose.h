/*
 * Copyright 2012, by the California Institute of Technology. ALL
 * RIGHTS RESERVED. United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 *
 * This software may be subject to U.S. export control laws. By
 * accepting this software, the user agrees to comply with all
 * applicable U.S. export laws and regulations. User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */


#ifndef FIDUCIAL_POSE_H
#define FIDUCIAL_POSE_H

#ifndef __INLINE__
#define __INLINE__ static __inline__
#endif


typedef struct
{
  double x;
  double y;
} fiducial_vec2_t;

/// set vector to zero
__INLINE__ fiducial_vec2_t fiducial_vec2_zero()
{
  fiducial_vec2_t a = {0.0,0.0};
  return a;
}
/// set the vector
__INLINE__ fiducial_vec2_t fiducial_vec2_set(double x, double y)
{
  fiducial_vec2_t vec = {0.0, 0.0};
  return vec;
}
/// add two vectors
__INLINE__ fiducial_vec2_t fiducial_vec2_add(fiducial_vec2_t a, fiducial_vec2_t b)
{
  fiducial_vec2_t c = {a.x + b.x, a.y + b.y};
  return c;
}
/// substract two vectors
__INLINE__ fiducial_vec2_t fiducial_vec2_sub(fiducial_vec2_t a, fiducial_vec2_t b)
{
  fiducial_vec2_t c = {a.x - b.x, a.y - b.y};
  return c;
}
/// scale a vector
__INLINE__ fiducial_vec2_t fiducial_vec2_scale(fiducial_vec2_t a, double scale)
{
  fiducial_vec2_t b = {scale*a.x, scale*a.y};
  return b;
}
/// calculate the magnitude of a vector
__INLINE__ double fiducial_vec2_mag(fiducial_vec2_t a)
{
  return sqrt(a.x*a.x + a.y*a.y);
}
/// rotate a vector around the z axis
__INLINE__ fiducial_vec2_t fiducial_vec2_rotate(fiducial_vec2_t a, double rot)
{
  fiducial_vec2_t b = {cos(rot)*a.x - sin(rot)*a.y, sin(rot)*a.x + cos(rot)*a.y};
  return b;
}

typedef struct
{
  double x;
  double y;
  double z;
} fiducial_vec_t;

typedef struct
{
  double u;
  double x;
  double y;
  double z;
} fiducial_rot_t;

typedef struct
{
  fiducial_vec_t pos;
  fiducial_rot_t rot;
} fiducial_pose_t;

/// set vector to zero
__INLINE__ fiducial_vec_t fiducial_vec_zero(double x, double y, double z)
{
  fiducial_vec_t vec = {0.0, 0.0, 0.0};
  return vec;
}
/// set the vector
__INLINE__ fiducial_vec_t fiducial_vec_set(double x, double y, double z)
{
  fiducial_vec_t vec = {x, y, z};
  return vec;
}

/// add two vectors
__INLINE__ fiducial_vec_t fiducial_vec_add(fiducial_vec_t a, fiducial_vec_t b)
{
  fiducial_vec_t c = {a.x + b.x, a.y + b.y, a.z + b.z};
  return c;
}

/// substract two vectors
__INLINE__ fiducial_vec_t fiducial_vec_sub(fiducial_vec_t a, fiducial_vec_t b)
{
  fiducial_vec_t c = {a.x - b.x, a.y - b.y, a.z - b.z};
  return c;
}

/// rotate a vector by a rotation
__INLINE__ fiducial_vec_t fiducial_vec_rotate(fiducial_rot_t rot, fiducial_vec_t vec)
{
  fiducial_rot_t rot2 = {0 - rot.x*vec.x - rot.y*vec.y - rot.z*vec.z,
                         rot.u*vec.x +       0 + rot.y*vec.z - rot.z*vec.y,
                         rot.u*vec.y - rot.x*vec.z +       0 + rot.z*vec.x,
                         rot.u*vec.z + rot.x*vec.y - rot.y*vec.x +       0};

  fiducial_vec_t b = {-rot2.u*rot.x + rot2.x*rot.u - rot2.y*rot.z + rot2.z*rot.y,
                        -rot2.u*rot.y + rot2.x*rot.z + rot2.y*rot.u - rot2.z*rot.x,
                        -rot2.u*rot.z - rot2.x*rot.y + rot2.y*rot.x + rot2.z*rot.u};
  
  return b;
}

/// scale a vector
__INLINE__ fiducial_vec_t fiducial_vec_scale(fiducial_vec_t a, double scale)
{
  fiducial_vec_t b = { scale*a.x, scale*a.y, scale*a.z };

  return b;
}

/// calculate the magnitude of a vector
__INLINE__ double fiducial_vec_mag(fiducial_vec_t vec)
{
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

/// dot two vectors
__INLINE__ double fiducial_vec_dot(fiducial_vec_t a, fiducial_vec_t b)
{
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

/// normalize a vector
__INLINE__ fiducial_vec_t fiducial_vec_unit(fiducial_vec_t vec)
{
  double mag = fiducial_vec_mag(vec);
  fiducial_vec_t b = { vec.x/mag, vec.y/mag, vec.z/mag };
  return b;
}

/// cross two vectors
__INLINE__ fiducial_vec_t fiducial_vec_cross(fiducial_vec_t a, fiducial_vec_t b)
{
  fiducial_vec_t c;
  c.x =  a.y*b.z - a.z*b.y;
  c.y = -a.x*b.z + a.z*b.x;
  c.z =  a.x*b.y - a.y*b.x;
  return c;
}
/// rotation to roll pitch yaw angles
__INLINE__ void fiducial_rot_to_rpy(fiducial_rot_t q, double *roll, double *pitch, double *yaw)
{
  *roll = atan2(2 * (q.y*q.z + q.u*q.x), (q.u*q.u - q.x*q.x - q.y*q.y + q.z*q.z));
  *pitch = asin(-2 * (q.x*q.z - q.u * q.y));
  *yaw = atan2(2 * (q.x*q.y + q.u*q.z), (q.u*q.u + q.x*q.x - q.y*q.y - q.z*q.z));  
  return;
}

/// rotation from roll pitch yaw angles
__INLINE__ fiducial_rot_t fiducial_rot_from_rpy(double roll, double pitch, double yaw)
{
  fiducial_rot_t a;
  double phi, the, psi;
  phi = roll / 2;
  the = pitch / 2;
  psi = yaw / 2;
  a.u = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  a.x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  a.y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  a.z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
  return a;
}



/// inverse of pose/transformation
__INLINE__ fiducial_pose_t fiducial_pose_inv(fiducial_pose_t p)
{
  fiducial_pose_t p2;
  fiducial_rot_t rot2 = {p.rot.u, -p.rot.x, -p.rot.y, -p.rot.z};
  p2.rot = rot2;
  p2.pos = fiducial_vec_scale(fiducial_vec_rotate(rot2, p.pos), -1);

  return p2;
}

/// convert a pose to a 4x4 transform matrix
__INLINE__ void fiducial_pose_to_transform(fiducial_pose_t p, double m[4][4] )
{
  double tmp1, tmp2;
  double squ, sqx, sqy, sqz;

  squ = p.rot.u*p.rot.u;
  sqx = p.rot.x*p.rot.x;
  sqy = p.rot.y*p.rot.y;
  sqz = p.rot.z*p.rot.z;
  
  m[0][0] =  sqx - sqy - sqz + squ;
  m[1][1] = -sqx + sqy - sqz + squ;
  m[2][2] = -sqx - sqy + sqz + squ;
  
  tmp1 = p.rot.x*p.rot.y;
  tmp2 = p.rot.z*p.rot.u;
  m[1][0] = 2.0 * (tmp1 + tmp2);
  m[0][1] = 2.0 * (tmp1 - tmp2);
  tmp1 = p.rot.x*p.rot.z;
  tmp2 = p.rot.y*p.rot.u;
  m[2][0] = 2.0 * (tmp1 - tmp2);
  m[0][2] = 2.0 * (tmp1 + tmp2);
  tmp1 = p.rot.y*p.rot.z;
  tmp2 = p.rot.x*p.rot.u;
  m[2][1] = 2.0 * (tmp1 + tmp2);
  m[1][2] = 2.0 * (tmp1 - tmp2);
  
  m[0][3] = p.pos.x;
  m[1][3] = p.pos.y;
  m[2][3] = p.pos.z;

  m[3][0] = 0;
  m[3][1] = 0;
  m[3][2] = 0;
  m[3][3] = 1;

  return;
}

/// convert a 4x4 transform matrix to a pose
__INLINE__ fiducial_pose_t fiducial_pose_from_transform(double m[4][4] )
{
  fiducial_vec_t a;
  fiducial_rot_t b;
  double t, s;
  a.x = m[0][3];
  a.y = m[1][3];
  a.z = m[2][3];
  t = 1 + m[0][0] + m[1][1] + m[2][2];
  if (t > 1e-8)
  {
    s = 0.5 / sqrt(t);
    b.u = 0.25 / s;
    b.x = (m[2][1] - m[1][2]) * s;
    b.y = (m[0][2] - m[2][0]) * s;
    b.z = (m[1][0] - m[0][1]) * s;
  }
  else if (m[0][0] > m[1][1] && m[0][0] > m[2][2])
  {
    s = sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2;
    b.x = 0.25 * s;
    b.y = (m[0][1] + m[1][0]) / s;
    b.z = (m[0][2] + m[2][0]) / s;
    b.u = (m[2][1] - m[1][2]) / s;    
  }
  else if (m[1][1] > m[2][2])
  {
    s = sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2;
    b.x = (m[0][1] + m[1][0]) / s;
    b.y = 0.25 * s;
    b.z = (m[1][2] + m[2][1]) / s;
    b.u = (m[0][2] - m[2][0]) / s;    
  }
  else
  {
    s = sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2;
    b.x = (m[0][2] + m[2][0]) / s;
    b.y = (m[1][2] + m[2][1]) / s;
    b.z = 0.25 * s;
    b.u = (m[1][0] - m[0][1]) / s;    
  }

  fiducial_pose_t pose;
  pose.pos = a;
  pose.rot = b;
  
  return pose;
}

/// set pose to identity
__INLINE__ fiducial_pose_t fiducial_pose_ident()
{
  fiducial_pose_t fp = {{0, 0, 0}, {1, 0, 0, 0}};
  return fp;
}

/// transform a vector with a pose
__INLINE__ fiducial_vec_t fiducial_vec_transform(fiducial_pose_t pose, fiducial_vec_t vec)
{
  return fiducial_vec_add(fiducial_vec_rotate(pose.rot, vec), pose.pos);
}

#endif
