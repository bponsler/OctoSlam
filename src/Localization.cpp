#include "Localization.h"
#include "OctoSlamCalcs.h"


Localization::Localization(ros::NodeHandle n)
    :
    node(n)
{
    ros::NodeHandle pn("~");

    // Grab input parameters
    max_iterations = pn.param("max_iterations", 5);
    const std::string scan_topic = pn.param("scan_topic", std::string("scan"));
    const std::string pose_topic = pn.param("pose_topic", std::string("pose"));

    // Grab initial pose from parameters
    current_pose.at(X) = pn.param("init_x", 0.0);
    current_pose.at(Y) = pn.param("init_y", 0.0);
    current_pose.at(Z) = pn.param("init_z", 0.0);
    current_pose.at(PHI) = pn.param("init_roll", 0.0);
    current_pose.at(PSI) = pn.param("init_pitch", 0.0);
    current_pose.at(GAMMA) = pn.param("init_yaw", 0.0);

    // Subscribe to scan and pose data
    scan_sub = node.subscribe(
        scan_topic, 1000, &Localization::scan_callback, this);
    aux_sub = node.subscribe(
        pose_topic, 1000, &Localization::data_callback, this);

    // Will need to initialize the octomap once I know whether I'll
    // be building the map on the go (actual SLAM) vs. loading a known map
    // Also goes with how does the Localization, mapping go together?
}

void
Localization::data_callback(geometry_msgs::Point pose)
{
    // Update the roll, pitch, and altitude from sensor data
    current_pose.at(PHI) = pose.x;
    current_pose.at(PSI) = pose.y;
    current_pose.at(Z) = pose.z;
}

void
Localization::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Everything is being done in floats, because that's what
    // the scan comes as :)
    Eigen::Vector3f T(
        current_pose.at(X),
        current_pose.at(Y),
        current_pose.at(GAMMA));

    // Apply equations 15 through 19 to the incoming laser scan
    std::vector<octomath::Vector3> t_scan = calculations::transform_scan(
        scan, current_pose);

    // Data needed for algorithm steps
    Eigen::Matrix3f Hessian;
    Eigen::Vector3f det;
    octomath::Vector3 endpoint;
    octomath::Vector3 map_values;
    std::vector<octomath::Vector3> closest; // Rounded voxels
    float mv, dMdx, dMdy, gammaP, mres;

    // This performs the logic of algorithm 3 from the paper
    for (int n = 0; n < max_iterations; ++n) {
        Hessian.setZero();
        det.setZero();

        // Handle all transformed laser scan points
        for (int i = 0; i < t_scan.size(); ++i) {
            endpoint = t_scan.at(i);

            // Compute distance between the voxel centers
            // (i.e., resolution of the octomap)
            mres = calculations::calc_mres(map, endpoint);

            // Round down to the voxel center
            closest = calculations::round_voxels(endpoint, mres);

            // Compute the interpolated map values
            map_values = calculations::calc_map_values(
                map, endpoint, closest, mres);

            // Grab specific components from map values computation
            mv = map_values.x();  // Equation 26
            dMdx = map_values.y();  // Equation 27
            dMdy = map_values.z();  // Equation 28

            // Compute gamma value needed for calculations
            gammaP = calculations::calc_gammaP(
                current_pose.at(GAMMA),
                endpoint.x(),
                endpoint.y(),
                dMdx,
                dMdy);

            // Update the hessian and the det (part of equation 29)
            // Note: this for loop performs the sums for hessian and
            // det for each laser scan point
            Hessian = Hessian + calculations::calc_hessian(dMdx, dMdy, gammaP);
            det = det + calculations::calc_det(mv, dMdx, dMdy, gammaP);
        }

        // Compute the full equation 29 from the paper
        T = T + (Hessian * det);
    }

    // Store the new pose
    current_pose.at(X) = T(0);
    current_pose.at(Y) = T(1);
    current_pose.at(GAMMA) = T(2);

    // TODO: publish new pose
}
