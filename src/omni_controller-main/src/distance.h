#pragma once

#include "utils.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>


using namespace std;
using namespace Eigen;



struct AxisAlignedBoundingBox
{
    double width;
    double depth;
    double height;
};

struct DistanceStruct
{
    double distance;
    Vector3d witnessA;
    Vector3d witnessB;
};

class PointCloud;


