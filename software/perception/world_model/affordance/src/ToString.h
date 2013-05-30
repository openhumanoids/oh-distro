#ifndef TO_STRING_H
#define TO_STRING_H

#include <Eigen/Core>
#include <kdl/frames.hpp>
#include <string>
#include <vector>

namespace affordance
{

class ToString
{
 public:

  template <class T> static std::string toStr(T t)
    {
      std::stringstream s;
      s << t;
      return s.str();
    }
  
  static std::string toStr(const KDL::Vector &v)
  {
    return std::string("(") + toStr(v.x()) + std::string(", ") 
      + toStr(v.y()) + std::string(", ") 
      + toStr(v.z()) + std::string(")");
  }
  
  static std::string toStr(const KDL::Rotation &r)
  {
    double q1,q2,q3,q4;
    r.GetQuaternion(q1,q2,q3,q4);
    return std::string("(") + toStr(q1) + std::string(", ") 
      + toStr(q2) + std::string(", ") 
      + toStr(q3)
      + std::string(", ")
      + toStr(q4) + std::string(")");
  }

  
  static std::string toStr(const KDL::Frame &frame)
  {
    return std::string("Position = ") + toStr(frame.p)
      + std::string("\t Rotation = ") + toStr(frame.M);
  }

  static std::vector<std::string> split(const std::string &input, char delim)
  {
    std::vector<std::string> result;
    std::stringstream stream(input);
    std::string nextWord;
    while(getline(stream,nextWord,delim))
      if( !nextWord.empty() )
        result.push_back(nextWord);
    return result;
  } 
}; //class ToString

/*
std::ostream& operator << (std::ostream &out, const KDL::Rotation &r)
{
  out << ToString::toStr(r);
  return out;
}

std::ostream& operator << (std::ostream &out, const KDL::Vector &v)
{
  out << ToString::toStr(v);
  return out;
}

std::ostream& operator <<(std::ostream& out, const KDL::Frame &frame)
{
  out << "Position = " << frame.p << "\t Rotation = " << frame.M;
  return out;
  }*/

} //namespace affordance

#endif //TO_STRING_H
