#include "pointmatcher/PointMatcher.h"

#include "boost/filesystem.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

void computeCloudsDistance (PM::ICP &icp, DP &cloud_ref, DP &data_out);

