/*
  Generic templated methods for converting from one point type to another.
  Works with c-arrays, c++ vectors, Eigen::Vector3*.  
  Pretty much anything of size 3 addressable by []

  Simple example:
    Vector3d in(1,2,3);
    float out[3];
    convert3(in,out);

  Vector example:
    vector<Vector3f> in;
    ... populate
    vector<vector<double> > out;
    convertVec3(in,out);

*/

#pragma once

#include <vector>

namespace drc{
namespace PointConvert{

  template<typename T1, typename T2>
  static void convert3(const T1& in, T2& out){
    out[0]=in[0];  
    out[1]=in[1];  
    out[2]=in[2];  
  }

  template<typename T1, typename T2>
  static void convertVec3(const std::vector<T1>& in, std::vector<T2>& out){
    out.resize(in.size());
    for(int i=0;i<in.size();i++) convert3(in[i],out[i]);
  }

  template<typename T1, typename T2>
  static void convertVec3(const std::vector<T1>& in, std::vector<std::vector<T2> >& out){
    out.resize(in.size());
    for(int i=0;i<in.size();i++) {
      out[i].resize(3);
      convert3(in[i],out[i]);
    }
  }

};
};
