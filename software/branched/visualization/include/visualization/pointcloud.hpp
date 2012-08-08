#ifndef _VS_VIS_POINTCLOUD_HPP
#define _VS_VIS_POINTCLOUD_HPP

#include <sys/types.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <isam/Point3d.h>

struct RGB {
  RGB(float r, float g, float b) : r(r), g(g), b(b) {}
  float r;
  float g;
  float b;
};

class PointCloud
{
public:
    PointCloud(int64_t utime) : m_utime(utime) {}
    void addPoint(float x, float y, float z) {
        m_points.push_back(isam::Point3d(x,y,z));
    }
    void addPoint(float x, float y, float z, float r, float g, float b) {
        m_points.push_back(isam::Point3d(x,y,z));
        m_colors.push_back(RGB(r,g,b));
    }

    const std::vector<isam::Point3d> & points() const { return m_points; }
    const std::vector<RGB> & colors() const { return m_colors; }
    size_t size() { return m_points.size(); }
    int64_t utime() { return m_utime; }

private:
    std::vector<isam::Point3d> m_points;
    std::vector<RGB> m_colors;
    int64_t m_utime;
};
typedef boost::shared_ptr<PointCloud> PointCloudPtr;

#endif
