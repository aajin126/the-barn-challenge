#ifndef NARROW_PASSAGE_DETECTOR_H
#define NARROW_PASSAGE_DETECTOR_H

#include <teb_local_planner/visualization.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <random>
#include <cmath>

class NarrowPassageDetector
{
public:
    NarrowPassageDetector(double thre, const nav_msgs::OccupancyGrid::ConstPtr& costmap, 
                          unsigned int num_samples, const nav_msgs::Path::ConstPtr& global_plan);

    std::vector<std::pair<geometry_msgs::Point, double>> detectNarrowPassages();

private:
    double thre_;
    nav_msgs::OccupancyGrid::ConstPtr costmap_;
    nav_msgs::Path::ConstPtr global_plan_;
    unsigned int num_samples_;

    double euclideanDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    std::vector<geometry_msgs::Point> generateSamples();
    double calculateAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& center);
    bool getObstaclePointsInCircle(const geometry_msgs::Point& center, double radius);
    std::pair<geometry_msgs::Point, double> findMedialBallRadius(const geometry_msgs::Point& point);
    bool isObstacleOrUnknown(double x, double y);
};

#endif // NARROW_PASSAGE_DETECTOR_H