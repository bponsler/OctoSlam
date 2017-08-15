// I have not test to see if the output is correct, although I feel pretty comfortable that it should be working correctly
#include "OctoSlamCalcs.h"
#include <math.h>

Eigen::Matrix3f
set_yaw(float yaw)
{
    // Matrix taken from equation 16 of the paper
    Eigen::Matrix3f rot;
	rot(0, 0) = cos(yaw);
	rot(0, 1) = -sin(yaw);
	rot(0, 2) = 0;
	rot(1, 0) = sin(yaw);
	rot(1, 1) = cos(yaw);
	rot(1, 2) = 0;
	rot(2, 0) = 0;
	rot(2, 1) = 0;
	rot(2, 2) = 1;
    return rot;
}

Eigen::Matrix3f
set_roll(float roll)
{
    // Matrix taken from equation 17 of the paper
    Eigen::Matrix3f rot;
	rot(0, 0) = cos(roll);
	rot(0, 1) = 0;
	rot(0, 2) = sin(roll);
	rot(1, 0) = 0;
	rot(1, 1) = 1;
	rot(1, 2) = 0;
	rot(2, 0) = -sin(roll);
	rot(2, 1) = 0;
	rot(2, 2) = cos(roll);
    return rot;
}

Eigen::Matrix3f
set_pitch(float pitch)
{
    // Matrix taken from equation 18 of the paper
    Eigen::Matrix3f rot;
	rot(0, 0) = cos(pitch);
	rot(0, 1) = -sin(pitch);
	rot(0, 2) = 0;
	rot(1, 0) = sin(pitch);
	rot(1, 1) = cos(pitch);
	rot(1, 2) = 0;
	rot(2, 0) = 0;
	rot(2, 1) = 0;
	rot(2, 2) = 1;
    return rot;
}

Eigen::Matrix<float, 3, 4>
set_translate(float x, float y, float z)
{
    // Matrix taken from equation 19 of the paper
    Eigen::Matrix<float, 3, 4> translate;
	translate(0, 0) = 1;
	translate(0, 1) = 0;
	translate(0, 2) = 0;
	translate(0, 3) = x;
	translate(1, 0) = 0;
	translate(1, 1) = 1;
	translate(1, 2) = 0;
	translate(1, 3) = y;
	translate(2, 0) = 0;
	translate(2, 1) = 0;
	translate(2, 2) = 1;
	translate(2, 3) = z;
    return translate;
}

// Check to make sure this compiles next time in lab
std::vector<octomath::Vector3>
calculations::transform_scan(
    const sensor_msgs::LaserScan::ConstPtr &scan,
    std::vector<float> current)
{
	float diff = scan->angle_max - scan->angle_min;
	float sizef = diff / scan->angle_increment;
	float angle =  scan->angle_min;
	int size = int(sizef);

    std::vector<octomath::Vector3> ret;

    // Grab matrices for equations 16 through 19
	Eigen::Matrix3f roll = set_roll(current.at(3));
	Eigen::Matrix3f pitch = set_pitch(current.at(4));
	Eigen::Matrix3f yaw = set_yaw(current.at(5));
	Eigen::Matrix<float, 3, 4> translate = set_translate(
        current.at(0), current.at(1), current.at(2));

    // Iterate over all of the laser scans
	for (int i = 0; i < size; i++) {
        // Handle scan points outside the min/max range
		if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max) {
            //ret.push_back(octomath::Vector3(-1, -1, -1));
            // I'm not entirely sure what I should do here. I think for now,
            // I'm just going to leave it as -1, and I will have to ignore
            // these points later in the algorithm
			continue;
		}
		float range = scan->ranges[i];
		Eigen::Vector3f endpoints;

        // Apply equation 15 from the paper
		endpoints(0) = range * cos(angle);
		endpoints(1) = range * sin(angle);
		endpoints(2) = 0;

		endpoints = yaw * endpoints;  // Apply equation 16 from the paper
		endpoints = roll * endpoints;  // Apply equation 17 from the paper
		endpoints = pitch * endpoints;  // Apply equation 18 from the paper

        // Convert end point to a 4x1 vector so it can be multipled
        // by the translation matrix
		Eigen::Vector4f addon;
		addon(0) = endpoints(0);
		addon(1) = endpoints(1);
		addon(2) = endpoints(2);
		addon(3) = 1;
		endpoints = translate * addon;  // Apply equation 19 from the paper

        // Keep track of the transformed point
		ret.push_back(octomath::Vector3(endpoints(0), endpoints(1), endpoints(2)));

        // Adjust to the next laser angle
		angle += scan->angle_increment;
	}

	return ret;
}
