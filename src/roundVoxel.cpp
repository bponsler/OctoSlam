#include "OctoSlamCalcs.h"


float
round(float in, float mres)
{
    float mod = fmod(in, mres);
    const float half_mres = (mres / 2);
    if (mod > half_mres) {
        return in + half_mres - mod;
    }
    return in - mod - half_mres;
}

std::vector<octomath::Vector3>
calculations::round_voxels(octomath::Vector3 endpoint, float mres)
{
    // Apply equation 23 from the paper to round
    // down to the next voxel center
    float v0x, v0y, v0z;
    v0x = round(endpoint.x(), mres);
    v0y = round(endpoint.y(), mres);
    v0z = round(endpoint.z(), mres);

    // Apply equation 25a from the paper
    float v1x, v1y, v1z;
    v1x = v0x + mres;
    v1y = v0y;
    v1z = v0z;

    // Apply equation 25b from the paper
    float v2x, v2y, v2z;
    v2x = v1x;
    v2y = v1y + mres;
    v2z = v1z;

    // Apply equation 25c from the paper
    float v3x, v3y, v3z;
    v3x = v2x + mres;
    v3y = v2y;
    v3z = v2z;

    // Create corresponding vectors
    octomath::Vector3 v0(v0x, v0y, v0z);
    octomath::Vector3 v1(v1x, v1y, v1z);
    octomath::Vector3 v2(v2x, v2y, v2z);
    octomath::Vector3 v3(v3x, v3y, v3z);

    // Combine into a vector of vectors
    std::vector<octomath::Vector3> ret;
    ret.push_back(v0);
    ret.push_back(v1);
    ret.push_back(v2);
    ret.push_back(v3);

    return ret;
}
