/*
 * PathPlanner.h
 *
 *  Created on: 29 Apr 2014
 *      Author: thomas
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <fstream>
#include <limits>
#include <iostream>
#include <queue>
#include <cmath>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>

class PathPlanner
{
    public:
        PathPlanner(std::string directory);
        virtual ~PathPlanner();

        cv::Mat3b getImageCopy()
        {
            return image;
        }

        std::vector<Eigen::Vector3f> getPath(Eigen::Vector3f start, Eigen::Vector3f end, cv::Mat3b * image = 0);

        Eigen::Vector3f imgToWorld(Eigen::Vector2i p);

        Eigen::Vector2i worldToImg(Eigen::Vector3f worldPoint);

    private:
        bool traceLine(Eigen::Vector2i p0, Eigen::Vector2i p1);

        bool aStar(Eigen::Vector2i p0, Eigen::Vector2i p1, std::vector<Eigen::Vector2i> & path);

        float heuristic(Eigen::Vector2i p0, Eigen::Vector2i p1);

        bool validPoint(Eigen::Vector2i p);

        std::vector<Eigen::Vector2i> getNeighbours(Eigen::Vector2i p);

        std::vector<Eigen::Vector2i> reconstructPath(std::map<std::pair<int, int>, std::pair<int, int> > & cameFrom,
                                                     std::pair<int, int> currentNode);

        std::vector<Eigen::Vector2i> smoothPath(std::vector<Eigen::Vector2i> & inputPath);

        bool findClosestPoint(Eigen::Vector3f start, Eigen::Vector3f & end, int radius);

        void drawPath(std::vector<Eigen::Vector3f> & path, cv::Mat3b & image);

        struct Comparator
        {
            bool operator()(const std::pair<int, Eigen::Vector2i> & lhs, const std::pair<int, Eigen::Vector2i> & rhs) const
            {
                return lhs.first > rhs.first;
            }
        };

        cv::Mat3b image;
        Eigen::Vector2f min, max, size;
        float zVal;

        int bot_in_pixels_;
};

#endif /* PATHPLANNER_H_ */
