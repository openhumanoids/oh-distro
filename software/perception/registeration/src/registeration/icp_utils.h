#include "pointmatcher/PointMatcher.h"

#include "boost/filesystem.hpp"

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <cassert>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <math.h>       /* cos, sin */

using namespace std;
using namespace PointMatcherSupport;

#define PI 3.14159265

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::Parameters Parameters;

void computeCloudsDistance (PM::ICP &icp, DP &cloud_ref, DP &data_out);

string readLineFromFile(string& filename, int line_number);

PM::TransformationParameters parseTransformation(string& transform,
                        const int cloudDimension);

