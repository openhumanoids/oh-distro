#include "PathPlanner.h"

bool gotFirst = false;

Eigen::Vector2i first, second;

PathPlanner * pathPlanner = 0;

cv::Mat3b plainCopy;
cv::Mat3b workingCopy;

static void drawOrigin()
{
    Eigen::Vector3f origin(0, 0, 0);

    Eigen::Vector2i originImg = pathPlanner->worldToImg(origin);

    cv::Point2i imgOrigin(originImg(0), originImg(1));

    cv::circle(workingCopy, imgOrigin, 4, CV_RGB(0, 255, 255), 2);
}

static void planPath()
{
    std::vector<Eigen::Vector3f> path = pathPlanner->getPath(pathPlanner->imgToWorld(first), pathPlanner->imgToWorld(second), &workingCopy);

    if(path.size() == 0)
    {
        cv::Point2i p0, p1;

        p0.x = first(0);
        p0.y = first(1);

        p1.x = second(0);
        p1.y = second(1);

        cv::line(workingCopy, p0, p1, CV_RGB(255, 0, 0), 1);
    }
}

static void onMouse(int event, int x, int y, int, void *)
{
    if(event != cv::EVENT_LBUTTONDOWN)
        return;

    if(!gotFirst)
    {
        plainCopy.copyTo(workingCopy);

        drawOrigin();

        first(0) = x;
        first(1) = y;
        gotFirst = true;

        cv::Point2i p0;
        p0.x = x;
        p0.y = y;

        cv::circle(workingCopy, p0, 4, CV_RGB(0, 255, 0), 2);
    }
    else if(gotFirst)
    {
        second(0) = x;
        second(1) = y;
        gotFirst = false;

        planPath();
    }

    Eigen::Vector2i pointImg(x, y);
    Eigen::Vector3f pointWorld = pathPlanner->imgToWorld(pointImg);

    std::cout << -pointWorld(0) << ", " << pointWorld(1) << std::endl;

    cv::imshow("Map", workingCopy);
}

int main(int argc, char ** argv)
{
    std::string directory;
    pcl::console::parse_argument(argc, argv, "-i", directory);

    assert(directory.length());

    pathPlanner = new PathPlanner(directory);

    plainCopy = pathPlanner->getImageCopy();

    plainCopy.copyTo(workingCopy);

    drawOrigin();

    cv::imshow("Map", workingCopy);

    cv::setMouseCallback("Map", onMouse, 0);

    char key = cv::waitKey(3);

    float x = 0;

    float y = 0;

    pcl::console::parse_argument(argc, argv, "-x", x);

    pcl::console::parse_argument(argc, argv, "-y", y);

    if(x != 0 || y != 0)
    {
        Eigen::Vector3f origin(0, 0, 0);

        Eigen::Vector2i originImg = pathPlanner->worldToImg(origin);

        first(0) = originImg(0);
        first(1) = originImg(1);

        Eigen::Vector3f point(-x, y, 0);
        Eigen::Vector2i imgPoint = pathPlanner->worldToImg(point);

        second(0) = imgPoint(0);
        second(1) = imgPoint(1);

        planPath();
    }

    while(key != 'q')
    {
        cv::imshow("Map", workingCopy);

        key = cv::waitKey(3);
    }
}
