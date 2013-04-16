#include "print_utils.hpp"
namespace opencv_utils {
    std::ostream& operator << (std::ostream& os, const cv::Rect& r) {
        return (os << "[x=" << r.x << ", y=" << r.y << ", w=" << r.width << ", h=" << r.height << "]");
    }
    
}
