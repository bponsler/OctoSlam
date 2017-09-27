#include "OctoSlamCalcs.h"
#include <math.h>


Eigen::Vector3f
calculations::calc_det(float mv, float dx, float dy, float gammaP)
{
    // Taken from equation 29 from the paper
    Eigen::Vector3f ret;
    ret(0) = dy;
    ret(1) = dx;
    ret(2) = gammaP;
    return (1.0f - mv) * ret;
}

Eigen::Matrix3f
calculations::calc_hessian(float dx, float dy, float gammaP)
{
    // Matrix taken from equation 29 from the paper
    Eigen::Matrix3f hessian;
    hessian(0, 0) = sqrt(dy);
    hessian(0, 1) = dy * dx;
    hessian(0, 2) = dy * gammaP;
    hessian(1, 0) = dy * dx;
    hessian(1, 1) = sqrt(dx);
    hessian(1, 2) = gammaP * dx;
    hessian(2, 0) = dy * gammaP;
    hessian(2, 1) = dy * dx;
    hessian(2, 2) = sqrt(gammaP);
    return hessian.inverse();
}

float
calculations::calc_gammaP(float gamma, float x, float y, float dx, float dy)
{
    // Compute equation 30 from the paper
    return (((-sin(gamma) * x) - (cos(gamma) * y)) * dy) +
        (((cos(gamma) * x) - (sin(gamma) * y)) * dx);
}

octomath::Vector3
calculations::calc_map_values(
    octomap::OcTree *map,
    octomath::Vector3 p,
    std::vector<octomath::Vector3> vectors,
    float mres)
{
    // Grab voxel position vectors
    octomath::Vector3 v0, v1, v2, v3;
    v0 = vectors.at(0);
    v1 = vectors.at(1);
    v2 = vectors.at(2);
    v3 = vectors.at(3);

    // Compute occupancy probability of vx stored in the octree node
    float Mv0 = map->search(v0)->getOccupancy();
    float Mv1 = map->search(v1)->getOccupancy();
    float Mv2 = map->search(v2)->getOccupancy();
    float Mv3 = map->search(v3)->getOccupancy();

    // Compute separate parts for equation 26 from the paper
    float mvPart0 = ((v3.y() - p.y()) / mres) *
        ((v1.x() - p.x()) / (v1.x() - v2.x())) * Mv0;
    float mvPart1 = ((v3.y() - p.y()) / mres) *
        ((p.x() - v2.x()) / (v1.x() - v2.x())) * Mv1;
    float mvPart2 = ((p.y() - v0.y()) / mres) *
        ((v1.x() - p.x()) / (v1.x() - v2.x())) * Mv2;
    float mvPart3 = ((p.y() - v0.y()) / mres) *
        ((p.x() - v2.x()) / (v1.x() - v2.x())) * Mv3;

    // Compute equation 26 from the paper
    float mv = mvPart0 + mvPart1 + mvPart2 + mvPart3;

    // Compute equation 26 from the paper
    float dMdx = (((v3.y() - p.y()) / mres) *
                  (Mv0 - Mv2)) + (((p.y() - v0.y()) / mres) * (Mv1 - Mv3));
    // Compute equation 27 from the paper
    float dMdy = (((v1.x() - p.x()) / mres) *
                  (Mv0 - Mv1)) + (((p.x() - v2.x()) / mres) * (Mv2 - Mv3));

    // Return the computations as a vector
    return octomath::Vector3(mv, dMdx, dMdy);
}

float
calculations::calc_mres(octomap::OcTree *map, octomath::Vector3 endpoint)
{
    // Search for the end point within the map
    octomap::OcTreeKey pointKey, itKey;
    pointKey = map->coordToKey(endpoint);
    if (map->search(endpoint) == NULL) return 0;

    // Compute the depth of the end point within the tree
    int depth;
    octomap::OcTree::leaf_iterator it;
    for (it = map->begin_leafs();it != map->end_leafs(); ++it) {
        itKey = it.getIndexKey();
        if (itKey == pointKey) {
            // using n = 0 as the base level
            depth = map->getTreeDepth() - it.getDepth();
            break;
        }
    }

    // Apply equation 24 from the paper
    return pow(2, depth) * map->getResolution();
}
