/*
 * PathPlanner.cpp
 *
 *  Created on: 29 Apr 2014
 *      Author: thomas
 */

#include "PathPlanner.h"

PathPlanner::PathPlanner(std::string directory)
{
    if(directory.at(directory.size() - 1) != '/')
    {
        directory.append("/");
    }

    std::string imageFile = directory + "map.png";
    std::string transformFile = directory + "transform.txt";

    image = cv::imread(imageFile);

    //As in FloorProjection.cpp

    const int scale = 100;
    float robotWidth = 0.395;
    int erosionPixels = (robotWidth * scale) / 2;

    //This magic number is the size of the robot in pixels, as set out in FloorProjection.cpp
    bot_in_pixels_ = (int) robotWidth*scale;

    cv::Mat1b grey;
    cv::Mat1b config;

    cv::cvtColor(image, grey, CV_RGB2GRAY);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionPixels + 1, 2 * erosionPixels + 1), cv::Point(erosionPixels, erosionPixels));

    cv::dilate(grey, config, element);

    for(int i = 0; i < image.rows; i++)
    {
        for(int j = 0; j < image.cols; j++)
        {
            if(image.at<cv::Vec<unsigned char, 3> >(i, j)[0] == 0 && config.at<char>(i, j) != 0)
            {
                image.at<cv::Vec<unsigned char, 3> >(i, j)[0] = 128;
                image.at<cv::Vec<unsigned char, 3> >(i, j)[1] = 128;
                image.at<cv::Vec<unsigned char, 3> >(i, j)[2] = 128;
            }
        }
    }

    std::ifstream file;
    std::string line;

    file.open(transformFile.c_str());

    double z, minx, miny, maxx, maxy, t0, t1, t2, t3, t4, t5, t6, t7, t8, t9;

    std::getline(file, line);

    int n = sscanf(line.c_str(), "%lg", &z);

    std::getline(file, line);

    n = sscanf(line.c_str(), "%lg %lg", &minx, &miny);

    std::getline(file, line);

    n = sscanf(line.c_str(), "%lg %lg", &maxx, &maxy);

    zVal = z;

    min(0) = minx;
    min(1) = miny;

    max(0) = maxx;
    max(1) = maxy;

    size = max - min;
}

PathPlanner::~PathPlanner()
{

}

std::vector<Eigen::Vector3f> PathPlanner::getPath(Eigen::Vector3f start, Eigen::Vector3f end, cv::Mat3b * image)
{
    std::vector<Eigen::Vector3f> path;
    std::vector<Eigen::Vector2i> pathImg;

    //Objects are outside the configuration space, so get the closest point
    Eigen::Vector2i lastPoint;
    if(!validPoint(worldToImg(end)))
    {
        if(!findClosestPoint(start, end, bot_in_pixels_))
        {
            return path;
        }
    }

    //Check if we can just go straight there unobstructed
    if(traceLine(worldToImg(start), worldToImg(end)))
    {
        path.push_back(start);
        path.push_back(end);
    }
    else if(aStar(worldToImg(start), worldToImg(end), pathImg))
    {
        pathImg = smoothPath(pathImg);

        for(size_t i = 0; i < pathImg.size(); i++)
        {
            path.push_back(imgToWorld(pathImg.at(i)));
        }
    }

    if(image)
    {
        drawPath(path, *image);
    }

    return path;
}

void PathPlanner::drawPath(std::vector<Eigen::Vector3f> & path, cv::Mat3b & image)
{
    if(path.size() != 0)
    {
        for(size_t i = 1; i < path.size(); i++)
        {
            cv::Point2i p0, p1;
            Eigen::Vector2i pe0 = worldToImg(path.at(i - 1));
            Eigen::Vector2i pe1 = worldToImg(path.at(i));

            p0.x = pe0(0);
            p0.y = pe0(1);

            p1.x = pe1(0);
            p1.y = pe1(1);

            cv::line(image, p0, p1, CV_RGB(0, 255, 0), 2);
        }

        for(size_t i = 1; i < path.size(); i++)
        {
            cv::Point2i p0, p1;
            Eigen::Vector2i pe0 = worldToImg(path.at(i - 1));
            Eigen::Vector2i pe1 = worldToImg(path.at(i));

            p0.x = pe0(0);
            p0.y = pe0(1);

            p1.x = pe1(0);
            p1.y = pe1(1);

            cv::circle(image, p0, 4, CV_RGB(0, 0, 255), 2);

            cv::circle(image, p1, 4, CV_RGB(0, 0, 255), 2);
        }
    }
}

bool PathPlanner::findClosestPoint(Eigen::Vector3f start, Eigen::Vector3f & end, int radius)
{
    Eigen::Vector2i imgPoint = worldToImg(end);

    std::vector<Eigen::Vector2i> points, validPoints;

    while(radius-- > 0)
    {
        //Collect valid pixels on the Bresenham circle
        int f = 1 - radius;
        int ddFx = 0;
        int ddFy = -2 * radius;
        int x = 0;
        int y = radius;

        points.push_back(Eigen::Vector2i(imgPoint(0), imgPoint(1) + radius));
        points.push_back(Eigen::Vector2i(imgPoint(0), imgPoint(1) - radius));
        points.push_back(Eigen::Vector2i(imgPoint(0) + radius, imgPoint(1)));
        points.push_back(Eigen::Vector2i(imgPoint(0) - radius, imgPoint(1)));

        while(x < y)
        {
            if(f >= 0)
            {
                y--;
                ddFy += 2;
                f += ddFy;
            }
            x++;
            ddFx += 2;
            f += ddFx + 1;
            points.push_back(Eigen::Vector2i(imgPoint(0) + x, imgPoint(1) + y));
            points.push_back(Eigen::Vector2i(imgPoint(0) - x, imgPoint(1) + y));
            points.push_back(Eigen::Vector2i(imgPoint(0) + x, imgPoint(1) - y));
            points.push_back(Eigen::Vector2i(imgPoint(0) - x, imgPoint(1) - y));
            points.push_back(Eigen::Vector2i(imgPoint(0) + y, imgPoint(1) + x));
            points.push_back(Eigen::Vector2i(imgPoint(0) - y, imgPoint(1) + x));
            points.push_back(Eigen::Vector2i(imgPoint(0) + y, imgPoint(1) - x));
            points.push_back(Eigen::Vector2i(imgPoint(0) - y, imgPoint(1) - x));
        }
    }

    for(size_t i = 0; i < points.size(); i++)
    {
        if(image.at<cv::Vec<unsigned char, 3> >(points.at(i)(1), points.at(i)(0))[0]
                        == 0)
        {
            validPoints.push_back(points.at(i));
        }
    }

    Eigen::Vector2i startPoint = worldToImg(start);
    float bestDistance = std::numeric_limits<float>::max();

    for(size_t i = 0; i < validPoints.size(); i++)
    {
        float distance = (startPoint.cast<float>()
                        - validPoints.at(i).cast<float>()).norm();

        if(distance < bestDistance)
        {
            bestDistance = distance;
            end = imgToWorld(validPoints.at(i));
        }
    }

    return validPoint(worldToImg(end));
}

Eigen::Vector3f PathPlanner::imgToWorld(Eigen::Vector2i p)
{
    Eigen::Vector3f worldPoint;

    worldPoint(0) = ((float) p(0) / (float) image.cols) * size(0) + min(0);
    worldPoint(1) = ((float) p(1) / (float) image.rows) * size(1) + min(1);
    worldPoint(2) = zVal;

    return worldPoint;
}

Eigen::Vector2i PathPlanner::worldToImg(Eigen::Vector3f worldPoint)
{
    Eigen::Vector2i point;

    point(0) = round((image.cols * (worldPoint(0) - min(0))) / size(0));
    point(1) = round((image.rows * (worldPoint(1) - min(1))) / size(1));

    return point;
}

bool PathPlanner::traceLine(Eigen::Vector2i p0, Eigen::Vector2i p1)
{
    int x1 = p0(0);
    int y1 = p0(1);

    int x2 = p1(0);
    int y2 = p1(1);

    if(x2 - x1 == 0)
    {
        for(int y = y1; y < y2; y++)
        {
            if(image.at<cv::Vec<unsigned char, 3> >(y, x1)[0] != 0)
            {
                return false;
            }
        }
    }
    else if(y2 - y1 == 0)
    {
        for(int x = x1; x < x2; x++)
        {
            if(image.at<cv::Vec<unsigned char, 3> >(y1, x)[0] != 0)
            {
                return false;
            }
        }
    }
    else
    {
        //Bresenham
        const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
        if(steep)
        {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }

        if(x1 > x2)
        {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        const float dx = x2 - x1;
        const float dy = fabs(y2 - y1);

        float error = dx / 2.0f;
        const int ystep = (y1 < y2) ? 1 : -1;
        int y = (int) y1;

        const int maxX = (int) x2;

        for(int x = (int) x1; x < maxX; x++)
        {
            if(steep)
            {
                if(image.at<cv::Vec<unsigned char, 3> >(x, y)[0] != 0)
                {
                    return false;
                }
            }
            else
            {
                if(image.at<cv::Vec<unsigned char, 3> >(y, x)[0] != 0)
                {
                    return false;
                }
            }

            error -= dy;
            if(error < 0)
            {
                y += ystep;
                error += dx;
            }
        }
    }

    return true;
}

float PathPlanner::heuristic(Eigen::Vector2i p0, Eigen::Vector2i p1)
{
    return sqrt(pow((float) (p0(0) - p1(0)), 2.0f)
                    + pow((float) (p0(1) - p1(1)), 2.0f));
}

bool PathPlanner::validPoint(Eigen::Vector2i p)
{
    return (p(0) >= 0 && p(0) < image.cols && p(1) >= 0 && p(1) < image.rows
                    && image.at<cv::Vec<unsigned char, 3> >(p(1), p(0))[0] == 0);
}

std::vector<Eigen::Vector2i> PathPlanner::getNeighbours(Eigen::Vector2i p)
{
    Eigen::Vector2i p0(p(0) + 1, p(1));
    Eigen::Vector2i p1(p(0) - 1, p(1));
    Eigen::Vector2i p2(p(0), p(1) + 1);
    Eigen::Vector2i p3(p(0), p(1) - 1);

    Eigen::Vector2i p4(p(0) + 1, p(1) + 1);
    Eigen::Vector2i p5(p(0) - 1, p(1) - 1);
    Eigen::Vector2i p6(p(0) - 1, p(1) + 1);
    Eigen::Vector2i p7(p(0) + 1, p(1) - 1);

    std::vector<Eigen::Vector2i> points;

    if(validPoint(p0))
        points.push_back(p0);
    if(validPoint(p1))
        points.push_back(p1);
    if(validPoint(p2))
        points.push_back(p2);
    if(validPoint(p3))
        points.push_back(p3);

    //Disallow walking through diagonal cracks
    if(validPoint(p4) && (validPoint(p0) || validPoint(p2)))
        points.push_back(p4);
    if(validPoint(p5) && (validPoint(p1) || validPoint(p3)))
        points.push_back(p5);
    if(validPoint(p6) && (validPoint(p1) || validPoint(p2)))
        points.push_back(p6);
    if(validPoint(p7) && (validPoint(p0) || validPoint(p3)))
        points.push_back(p7);

    return points;
}

std::vector<Eigen::Vector2i> PathPlanner::reconstructPath(std::map<
                std::pair<int, int>, std::pair<int, int> > & cameFrom, std::pair<
                int, int> currentNode)
{
    std::vector<Eigen::Vector2i> path;

    if(cameFrom.find(currentNode) != cameFrom.end())
    {
        path = reconstructPath(cameFrom, cameFrom[currentNode]);
        path.push_back(Eigen::Vector2i(currentNode.first, currentNode.second));
        return path;
    }
    else
    {
        path.push_back(Eigen::Vector2i(currentNode.first, currentNode.second));
        return path;
    }
}

bool PathPlanner::aStar(Eigen::Vector2i start, Eigen::Vector2i end, std::vector<
                Eigen::Vector2i> & path)
{
    std::map<std::pair<int, int>, std::pair<int, int> > cameFrom;

    bool ** closedSet = new bool *[image.cols];

    for(int i = 0; i < image.cols; i++)
    {
        closedSet[i] = new bool[image.rows];

        for(int j = 0; j < image.rows; j++)
        {
            closedSet[i][j] = false;
        }
    }

    bool ** openSet = new bool *[image.cols];

    for(int i = 0; i < image.cols; i++)
    {
        openSet[i] = new bool[image.rows];

        for(int j = 0; j < image.rows; j++)
        {
            openSet[i][j] = false;
        }
    }

    float ** gScore = new float *[image.cols];

    for(int i = 0; i < image.cols; i++)
    {
        gScore[i] = new float[image.rows];

        for(int j = 0; j < image.rows; j++)
        {
            gScore[i][j] = 0;
        }
    }

    float ** fScore = new float *[image.cols];

    for(int i = 0; i < image.cols; i++)
    {
        fScore[i] = new float[image.rows];

        for(int j = 0; j < image.rows; j++)
        {
            fScore[i][j] = 0;
        }
    }

    openSet[start(0)][start(1)] = true;
    gScore[start(0)][start(1)] = 0;
    fScore[start(0)][start(1)] = gScore[start(0)][start(1)]
                    + heuristic(start, end);

    std::priority_queue<std::pair<float, Eigen::Vector2i>,
                    std::vector<std::pair<int, Eigen::Vector2i> >,
                    PathPlanner::Comparator> queue;

    queue.push(std::pair<float, Eigen::Vector2i>(0, start));

    while(!queue.empty())
    {
        Eigen::Vector2i current = queue.top().second;
        queue.pop();

        int x = current(0);
        int y = current(1);

        if(current == end)
        {
            path =
                            reconstructPath(cameFrom, std::pair<int, int>(end(0), end(1)));

            for(int i = 0; i < image.cols; i++)
            {
                delete[] closedSet[i];
            }

            delete[] closedSet;

            for(int i = 0; i < image.cols; i++)
            {
                delete[] openSet[i];
            }

            delete[] openSet;

            for(int i = 0; i < image.cols; i++)
            {
                delete[] gScore[i];
            }

            delete[] gScore;

            for(int i = 0; i < image.cols; i++)
            {
                delete[] fScore[i];
            }

            delete[] fScore;

            return true;
        }

        openSet[x][y] = false;
        closedSet[x][y] = true;

        std::vector<Eigen::Vector2i> neighbours = getNeighbours(current);

        for(size_t i = 0; i < neighbours.size(); i++)
        {
            int nx = neighbours.at(i)(0);
            int ny = neighbours.at(i)(1);

            if(closedSet[nx][ny])
                continue;

            float gTentative = gScore[x][y]
                            + heuristic(current, neighbours.at(i));

            if(!openSet[nx][ny] || gTentative < gScore[nx][ny])
            {
                cameFrom[std::pair<int, int>(neighbours.at(i)(0), neighbours.at(i)(1))] =
                                std::pair<int, int>(current(0), current(1));
                gScore[nx][ny] = gTentative;
                fScore[nx][ny] = gScore[nx][ny]
                                + heuristic(neighbours.at(i), end);
                openSet[nx][ny] = true;
                queue.push(std::pair<float, Eigen::Vector2i>(fScore[nx][ny], neighbours.at(i)));
            }
        }
    }

    for(int i = 0; i < image.cols; i++)
    {
        delete[] closedSet[i];
    }

    delete[] closedSet;

    for(int i = 0; i < image.cols; i++)
    {
        delete[] openSet[i];
    }

    delete[] openSet;

    for(int i = 0; i < image.cols; i++)
    {
        delete[] gScore[i];
    }

    delete[] gScore;

    for(int i = 0; i < image.cols; i++)
    {
        delete[] fScore[i];
    }

    delete[] fScore;

    return false;
}

std::vector<Eigen::Vector2i> PathPlanner::smoothPath(std::vector<Eigen::Vector2i> & inputPath)
{
    if(inputPath.size() <= 2)
    {
        return inputPath;
    }

    std::vector<Eigen::Vector2i> outputPath;

    outputPath.push_back(inputPath.at(0));

    int inputIndex = 2;

    while(inputIndex < inputPath.size() - 1)
    {
        if(!traceLine(outputPath.back(), inputPath[inputIndex]))
        {
            outputPath.push_back(inputPath[inputIndex - 1]);
        }
        inputIndex++;
    }

    outputPath.push_back(inputPath.back());

    return outputPath;
}
