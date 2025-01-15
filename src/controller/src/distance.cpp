#include "utils.h"
#include "distance.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>

#include <ros/ros.h>
#include <ros/service.h>

using namespace std;
using namespace Eigen;

// Auxiliary functions
#define PI 3.1415926
#define SQRTHALFPI 1.2533141
#define SQRT2 1.4142135
#define CONSTJA 2.7889

double fun_J(double u)
{
    return CONSTJA / ((CONSTJA - 1) * sqrt(PI * u * u) + sqrt(PI * u * u + CONSTJA * CONSTJA));
}

double fun_I0_hat(double u)
{
    return pow(1 + 0.25 * u * u, -0.25) * (1 + 0.24273 * u * u) / (1 + 0.43023 * u * u);
}

double fun_f(double nu, double rho)
{
    double A1 = exp(-0.5 * (rho - nu) * (rho - nu));
    double A2 = exp(-0.5 * (rho + nu) * (rho + nu));
    return rho * (A1 + A2) * fun_I0_hat(rho * nu);
}

double fun_f_hat(double nu, double rho, double rhobar)
{
    double A1 = exp(-0.5 * (rho - nu) * (rho - nu) + 0.5 * (rhobar - nu) * (rhobar - nu));
    double A2 = exp(-0.5 * (rho + nu) * (rho + nu) + 0.5 * (rhobar - nu) * (rhobar - nu));
    return rho * (A1 + A2) * fun_I0_hat(rho * nu);
}

double max(double a, double b)
{
    if (a >= b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

double min(double a, double b)
{
    if (a >= b)
    {
        return b;
    }
    else
    {
        return a;
    }
}

double Int(double v, double h, double L)
{
    if (abs(v) <= L)
    {
        double A1 = exp(-(L - v) * (L - v) / (2 * h * h)) * fun_J((v - L) / (SQRT2 * h));
        double A2 = exp(-(L + v) * (L + v) / (2 * h * h)) * fun_J((v + L) / (SQRT2 * h));
        return -h * h * log(SQRTHALFPI * (h / (2 * L)) * (2 - A1 - A2));
    }
    else
    {
        // The function is even
        v = abs(v);

        double A1 = fun_J((v - L) / (SQRT2 * h));
        double A2 = exp(-2 * L * v / (h * h)) * fun_J((v + L) / (SQRT2 * h));
        return 0.5 * (v - L) * (v - L) - h * h * log(SQRTHALFPI * (h / (2 * L)) * (A1 - A2));
    }
}

 
