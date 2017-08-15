#ifndef LOCALIZATION
#define LOCALIZATION

#include <geometry_msgs/Point.h>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>


class Localization
{
public:
    Localization(ros::NodeHandle n);
    void data_callback(geometry_msgs::Point pose);
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
    static const int X = 0; // Not known
    static const int Y = 1; // Not known
    static const int Z = 2; // Known from altitude data
    static const int PHI = 3; // Known from roll data
    static const int PSI = 4; // Known from pitch data
    static const int GAMMA = 5;  // Not known

    /** The node handle */
    ros::NodeHandle node;

    /** Subscribe to laser scan data. */
    ros::Subscriber scan_sub;

    /** Subscribe to known position data (for roll, pitch, and z) */
    ros::Subscriber aux_sub;

    /** Store the current pose. */
    std::vector<float> current_pose;

    /** The current map. */
    octomap::OcTree* map;

    /** The maximum number of iterations. */
    int max_iterations;
};


#endif
