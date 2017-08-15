#include "Localization.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "Localization");
    ros::NodeHandle n;
    Localization l(n);
    ros::spin();
}
