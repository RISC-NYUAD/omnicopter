
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

#include "quadprogpp/src/QuadProg++.hh"
#include "quadprogpp/src/QuadProg++.cc"

using namespace std;
using namespace Eigen;

Matrix4d Utils::rotx(double theta)
{
    double cost = cos(theta);
    double sint = sin(theta);

    Matrix4d rot;
    rot << 1, 0, 0, 0,
        0, cost, -sint, 0,
        0, sint, cost, 0,
        0, 0, 0, 1;

    return rot;
}
Matrix4d Utils::roty(double theta)
{
    double cost = cos(theta);
    double sint = sin(theta);

    Matrix4d rot;
    rot << cost, 0, sint, 0,
        0, 1, 0, 0,
        -sint, 0, cost, 0,
        0, 0, 0, 1;

    return rot;
}
Matrix4d Utils::rotz(double theta)
{
    double cost = cos(theta);
    double sint = sin(theta);

    Matrix4d rot;
    rot << cost, -sint, 0, 0,
        sint, cost, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return rot;
}
Matrix3d Utils::rotaxis(double theta, Vector3d axis)
{
    Vector3d axisn = axis.normalized();
    Matrix3d K = Utils::S(axis);
    return Matrix3d::Identity(3, 3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
}
Matrix4d Utils::trn(Vector3d v)
{

    Matrix4d trn;
    trn << 1, 0, 0, v(0),
        0, 1, 0, v(1),
        0, 0, 1, v(2),
        0, 0, 0, 1;

    return trn;
}
Matrix4d Utils::trn(double x, double y, double z)
{

    Matrix4d trn;
    trn << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;

    return trn;
}
Matrix3d Utils::S(Vector3d v)
{
    Matrix3d res = Matrix3d::Zero();

    res(0, 1) = -v(2);
    res(1, 0) = v(2);
    res(0, 2) = v(1);
    res(2, 0) = -v(1);
    res(1, 2) = -v(0);
    res(2, 1) = v(0);

    return res;
}
Vector3d Utils::cross(Vector3d a, Vector3d b)
{
    return Utils::S(a) * b;
}
MatrixXd Utils::pinv(MatrixXd M, double eps)
{
    int m = M.cols();
    MatrixXd MT = M.transpose();
    return (eps * MatrixXd::Identity(m, m) + MT * M).inverse() * MT;
}
VectorXd Utils::vecPow(VectorXd v, int indStart, int indEnd, double h)
{
    // int n = v.rows();
    VectorXd w = v;

    for (int i = indStart; i < indEnd; i++)
    {
        w[i] = (v[i] > 0 ? 1 : -1) * pow(abs(v[i]), h);
    }

    return w;
}
double Utils::rand(double vMin, double vMax)
{
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(vMin, vMax);

    return dist(mt);
}
VectorXd Utils::randVec(int n, double vMin, double vMax)
{
    VectorXd v(n);

    for (int i = 0; i < n; i++)
    {
        v[i] = rand(vMin, vMax);
    }

    return v;
}
Vector3d Utils::rpy(Matrix4d rot)
{
    double cos_pitch = sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2));
    double sin_pitch = -rot(2, 0);
    Vector3d angles = Vector3d::Zero();

    if (cos_pitch > 0.001)
    {
        double roll = atan2(rot(2, 1), rot(2, 2));
        double pitch = atan2(-rot(2, 0), cos_pitch);
        double yaw = atan2(rot(1, 0), rot(0, 0));
        angles << roll, pitch, yaw;
        return angles;
    }
    else
    {
        if (sin_pitch > 0)
        {
            double pitch = M_PI / 2;
            double yaw = 0;
            double roll = atan2(rot(0, 1), rot(0, 2));
            angles << roll, pitch, yaw;
            return angles;
        }
        else
        {
            double pitch = -M_PI / 2;
            double yaw = 0;
            double roll = atan2(-rot(0, 1), -rot(0, 2));
            angles << roll, pitch, yaw;
            return angles;
        }
    }
}

vector<VectorXd> Utils::upsample(vector<VectorXd> points, double dist)
{
    vector<VectorXd> upsampledPoints;
    VectorXd currentPoint;
    VectorXd nextPoint;
    VectorXd dir;
    double curDist;
    int jmax;

    for (int i = 1; i < points.size(); i++)
    {
        currentPoint = points[i - 1];
        nextPoint = points[i];
        curDist = (currentPoint - nextPoint).norm();

        jmax = (int)floor(curDist / dist) - 1;

        dir = (nextPoint - currentPoint) / (0.0000001 + curDist);

        for (int j = 0; j <= jmax; j++)
        {
            upsampledPoints.push_back(currentPoint + ((double)j) * dist * dir);
        }
    }
    upsampledPoints.push_back(nextPoint);

    return upsampledPoints;
}

VectorFieldResult Utils::vectorField(VectorXd q, vector<VectorXd> points, double alpha, bool openCurve, double percentLengthStop)
{

    // Find the closest point in the curve
    double dmin = (points[0] - q).norm();
    double dminTemp;
    int ind = 0;
    vector<double> s;
    s.push_back(0);

    for (int i = 0; i < points.size(); i++)
    {
        dminTemp = (points[i] - q).norm();
        if (dminTemp < dmin)
        {
            dmin = dminTemp;
            ind = i;
        }
        if (i > 0)
            s.push_back(s[s.size() - 1] + (points[i] - points[i - 1]).norm());
    }

    VectorXd pi = points[ind];

    // Compute the normal vector
    VectorXd N = (pi - q) / ((pi - q).norm() + 0.00001);

    // Compute the tangent vector
    VectorXd T;

    if (openCurve)
    {
        if (ind == 0)
            T = (points[1] - points[0]).normalized();
        else
            T = (points[ind] - points[ind - 1]).normalized();
    }
    else
    {
        if (ind == 0)
            T = (points[0] - points[points.size() - 1]).normalized();
        else
            T = (points[ind] - points[ind - 1]).normalized();
    }

    // Compute the G and H gains
    double G = (2 / M_PI) * atan(alpha * sqrt(dmin));
    double H = sqrt(1 - 0.9999 * G * G);

    // Compute the final vector field:
    VectorXd v = G * N + H * T;

    // Scale if the curve is open
    double sCurrent = s[ind];
    double sMaximum = s[s.size() - 1];
    double mult;
    if (openCurve)
    {
        mult = sqrt(abs((1 - (sCurrent - percentLengthStop * sMaximum) / ((1 - percentLengthStop) * sMaximum))));
        if (sCurrent > percentLengthStop * sMaximum)
            v = mult * v;
    }

    // if(openCurve)
    //     cout<<dmin<<std::endl;

    VectorFieldResult vfr;

    vfr.vector = v;
    vfr.distance = dmin;
    vfr.index = ind;

    return vfr;
}

VectorXd Utils::solveQP(MatrixXd H, VectorXd f, MatrixXd A, VectorXd b, MatrixXd Aeq, VectorXd beq)
{
    // Solve min_u (u'*H*u)/2 + f'*u
    // such that:
    // A*u >= b
    // Aeq*u = beq
    // The function assumes that H is a positive definite function (the problem is strictly convex)

    int n = H.rows();

    if (A.rows() == 0 && A.cols() == 0)
    {
        A = MatrixXd::Zero(0, n);
        b = VectorXd::Zero(0);
    }

    if (Aeq.rows() == 0 && Aeq.cols() == 0)
    {
        Aeq = MatrixXd::Zero(0, n);
        beq = VectorXd::Zero(0);
    }

    int meq = Aeq.rows();
    int mineq = A.rows();

    quadprogpp::Matrix<double> H_aux, Aeq_aux, A_aux;
    quadprogpp::Vector<double> f_aux, beq_aux, b_aux, u_aux;

    H_aux.resize(n, n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            H_aux[i][j] = H(i, j);

    f_aux.resize(n);
    for (int i = 0; i < n; i++)
        f_aux[i] = f[i];

    Aeq_aux.resize(n, meq);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < meq; j++)
            Aeq_aux[i][j] = Aeq(j, i);

    beq_aux.resize(meq);
    for (int j = 0; j < meq; j++)
        beq_aux[j] = -beq[j];

    A_aux.resize(n, mineq);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < mineq; j++)
            A_aux[i][j] = A(j, i);

    b_aux.resize(mineq);
    for (int j = 0; j < mineq; j++)
        b_aux[j] = -b[j];

    u_aux.resize(n);

    double val = solve_quadprog(H_aux, f_aux, Aeq_aux, beq_aux, A_aux, b_aux, u_aux);
	
    if (val > 1.0E50)
    {
        // Problem is unfeasible
        VectorXd u(0);
        return u;
    }
    else
    {
        // Problem is feasible
        VectorXd u(n);

        for (int i = 0; i < n; i++)
            u[i] = u_aux[i];

        return u;
    }
}

MatrixXd Utils::matrixVertStack(MatrixXd A1, MatrixXd A2)
{
    MatrixXd A(A1.rows() + A2.rows(), A1.cols());
    A << A1, A2;
    return A;
}

VectorXd Utils::vectorVertStack(VectorXd v1, VectorXd v2)
{
    VectorXd v(v1.rows() + v2.rows());
    v << v1, v2;
    return v;
}

VectorXd Utils::vectorVertStack(double v1, VectorXd v2)
{
    VectorXd v(1 + v2.rows());
    v << v1, v2;
    return v;
}

VectorXd Utils::vectorVertStack(VectorXd v1, double v2)
{
    VectorXd v(v1.rows() + 1);
    v << v1, v2;
    return v;
}

VectorXd Utils::vectorVertStack(double v1, double v2)
{
    VectorXd v(2);
    v << v1, v2;
    return v;
}

string Utils::printNumber(double x, int nochar)
{
    double P1 = pow(10, nochar-3);
    double P2 = 1/P1;

    double y = P2 * round(x * P1);
    string str;
    if (x >= 0)
        str = " " + std::to_string(y).substr(0, nochar-1);
    else
        str = std::to_string(y).substr(0, nochar-1);

    while (str.size() < nochar)
        str += "0";

    return str;
}


string Utils::printVector(VectorXd v, int nochar)
{
    string str="[";
    for(int i=0; i < v.rows()-1; i++)
        str+= printNumber(v[i], nochar)+", ";

    str+= printNumber(v[v.rows()-1], nochar)+"]";

    return str;
}

string Utils::printVectorOctave(VectorXd v, int nochar)
{
    string str="[";
    for(int i=0; i < v.rows()-1; i++)
        str+= printNumber(v[i], nochar)+" ";

    str+= printNumber(v[v.rows()-1], nochar)+"]";

    return str;
}

string Utils::printMatrix(MatrixXd M, int nochar)
{
    string str="";

    for(int i=0; i < M.rows(); i++)
    {
        str += (i==0) ? "[" : " ";
        for(int j=0; j < M.cols(); j++)
            str += Utils::printNumber(M(i,j), nochar)+"  ";

        str += (i==M.rows()-1) ? "]" : "\n";
        
    }

    return str;
}

string Utils::printMatrixPython(MatrixXd M, int nochar)
{
    string str="np.matrix([";

    for(int i=0; i < M.rows(); i++)
    {
        str += "[";
        for(int j=0; j < M.cols(); j++)
            str += Utils::printNumber(M(i,j),nochar)+((j==M.cols()-1) ? " " : ", ");

        str += (i==M.rows()-1) ? "]" : "], ";
        
    }

    return str+"])";
}

double Utils::softMin(vector<double> v, double h)
{
    double valMin=100000;
    double sum=0;

    for(int i=0; i < v.size(); i++)
        valMin = min(valMin, v[i]);

     for(int i=0; i < v.size(); i++)
     {
        sum+=exp(-(v[i]-valMin)/h);
     }  

     return valMin - h*log(sum/( (double) v.size()));
}

MatrixXd Utils::nullSpace(MatrixXd A)
{
    FullPivLU<MatrixXd> lu(A);
    MatrixXd Anull =  lu.kernel();

    if(Anull.cols()==1 && Anull.norm()<=0.001)
        return MatrixXd::Zero(A.cols(),0);
    else
        return Anull;
}

VectorXd Utils::hierarchicalSolve(vector<MatrixXd> A, vector<VectorXd> b, double eps)
{
    VectorXd xsol = Utils::pinv(A[0], eps) * b[0];
    MatrixXd nullA = Utils::nullSpace(A[0]);

    if(nullA.cols()==0 || A.size() == 1)
        return xsol;
    else
    {
        vector<MatrixXd> Anew;
        vector<VectorXd> bnew;
        for(int i = 1; i < A.size(); i++)
        {
            Anew.push_back(A[i]*nullA);
            bnew.push_back(b[i] - A[i]*xsol);
        }
        VectorXd ysol = Utils::hierarchicalSolve(Anew, bnew, eps);
        return xsol + nullA*ysol;
    }
    
}

SoftSelectMinResult Utils::softSelectMin(vector<double> v, vector<VectorXd> vVec, double h)
{
    double valMin=100000;
    double sum=0;
    double tempVal;
    VectorXd sumv = VectorXd::Zero(vVec[0].rows());

    for(int i=0; i < v.size(); i++)
        valMin = min(valMin, v[i]);

     for(int i=0; i < v.size(); i++)
     {
        tempVal = exp(-(v[i]-valMin)/h);
        sum+=tempVal;
        sumv+= tempVal * vVec[i];
     }  

     SoftSelectMinResult ssmr;

     ssmr.selected = sumv/sum;
     ssmr.residue = -h*log(sum/( (double) v.size()));
     ssmr.trueMin = valMin;
     ssmr.softMin = ssmr.trueMin + ssmr.residue;

     return ssmr;
}


MatrixXd Utils::optimalTrajFromWay(VectorXd x, VectorXd y, double v_max, double a_max, Vector2d v_start, Vector2d a_start, int n, VectorXd weights, int last_point,
MatrixXd& pp, MatrixXd& pd, MatrixXd& pdd, MatrixXd& Xd,  MatrixXd& Xdd, double& t_final) {
    VectorXd X;
    VectorXd tSamples;
    pp = MatrixXd::Zero(2,(n+1));
    pd = MatrixXd::Zero(2,(n));
    pdd = MatrixXd::Zero(2,(n-1));

    int k = x.size();
    if (x.size() != y.size()) {
        std::cout << "x and y vectors must be of the same size" << std::endl;
        return X;
    }
    X = VectorXd::Zero(k*2);
    VectorXd s = VectorXd::LinSpaced(k, 0, 1);

    MatrixXd Ax;
    Ax = MatrixXd::Ones(k,n+1);

    for (int i = n-1; i >=0 ; i--) {
        Ax.col(i) = (Ax.col(i+1).array() * s.array()).matrix();
    }

    Eigen::VectorXd x_diff(x.size()-1);
    Eigen::VectorXd y_diff(y.size()-1);
    x_diff = x.head(k-1) - x.tail(k-1);
    y_diff = y.head(k-1) - y.tail(k-1);
    double T_max = 2 * std::max(x_diff.cwiseAbs().mean(), y_diff.cwiseAbs().mean()) * k / v_max * 10;
    VectorXd T = VectorXd::LinSpaced(10, 2 * T_max / 10, T_max);

    MatrixXd vals = MatrixXd::Zero(2, T.size());
    int nPlus1 = n + 1;

    MatrixXd Axx = MatrixXd::Zero(k, nPlus1);
    MatrixXd Axxd = MatrixXd::Zero(k, n);
    MatrixXd Axxdd = MatrixXd::Zero(k, n - 1);

    MatrixXd A = MatrixXd::Zero(2 * k, 2 * n-4);
    
    VectorXd sumX(k);
    MatrixXd sumX_(k,3);
    Vector3d initialX(3);
    Vector2d V, Acc;
    V << v_start;
    Acc << a_start;
    double t, val_t;
    double val_min = 1000000;

    VectorXd mult(nPlus1);
    for (int i = 0; i < T.size(); ++i) {
        t = T(i);

	for (int j = 0; j <= n; j++){
		mult[n-j] = pow(t,j);
		}
		
	for (int j = 0; j <= n; j++){
		Axx.col(j) = Ax.col(j).array() * mult(j);
	}

        for (int j = 0; j < n; j++){
		Axxd.col(j) = Axx.col(j+1).array() * (n-j);
		if (j<n-1)
		Axxdd.col(j) = Axx.col(j+2).array() * (n-j-1) * (n-j);
	}


	A << Axx.leftCols(n - 2), MatrixXd::Zero(k, n-2),
	MatrixXd::Zero(k, n-2), Axx.leftCols(n - 2);
		
        initialX << 0.5 * Acc(0), V(0), x(0);
	sumX_ << (Axx.rightCols(3).array().rowwise() * initialX.transpose().array()).matrix();

	sumX << sumX_.rowwise().sum();
        VectorXd x_ = x - sumX;
        initialX << 0.5 * Acc(1), V(1), y(0);
        sumX_ << Axx.rightCols(3).array().rowwise() * initialX.transpose().array();
	sumX << sumX_.rowwise().sum();
        VectorXd y_ = y - sumX;
	//if (i==0)
	//std::cout << Axxdd << std::endl;
        VectorXd V_bound_1 = Axxd.rightCols(2) * Vector2d(0.5 * Acc(0), V(0));
        VectorXd A_bound_1 = 0.5 * Axxdd.rightCols(1)* Acc(0);
        
        VectorXd V_bound_2 = Axxd.rightCols(2) * Vector2d(0.5 * Acc(1), V(1));
        VectorXd A_bound_2 = 0.5 * Axxdd.rightCols(1) * Acc(1);

        MatrixXd A_constraint(8 * k, 2 * n - 4);
        A_constraint << Axxd.leftCols(n - 2), MatrixXd::Zero(k, n-2),
        MatrixXd::Zero(k, n - 2), Axxd.leftCols(n - 2),
        -Axxd.leftCols(n - 2), MatrixXd::Zero(k, n-2),
         MatrixXd::Zero(k, n - 2),-Axxd.leftCols(n - 2),
         Axxdd.leftCols(n - 2), MatrixXd::Zero(k, n-2), 
         MatrixXd::Zero(k, n - 2), Axxdd.leftCols(n - 2),
         -Axxdd.leftCols(n - 2), MatrixXd::Zero(k, n-2), 
         MatrixXd::Zero(k, n - 2), -Axxdd.leftCols(n - 2);

        VectorXd b_constraint(8 * k);
        b_constraint << -V_bound_1 + v_max * VectorXd::Ones(k),
                        -V_bound_2 + v_max * VectorXd::Ones(k),
                        V_bound_1 + v_max * VectorXd::Ones(k),
                        V_bound_2 + v_max * VectorXd::Ones(k),
                        -A_bound_1 + a_max * VectorXd::Ones(k),
                        -A_bound_2 + a_max * VectorXd::Ones(k),
                        A_bound_1 + a_max * VectorXd::Ones(k),
                        A_bound_2 + a_max * VectorXd::Ones(k);

        MatrixXd A_equality(6,A_constraint.cols());
        VectorXd b_equality(6);
	Eigen::VectorXd coeff;
	
	Eigen::VectorXd f(A.rows()), f_(x_.size()*2);
	f_ << x_, y_;

	f =  A.transpose()* f_;
	//if (i==0){
	//cout <<A_constraint << std::endl;
	//cout << b_constraint << std::endl;}
        if (last_point == 0) {
            coeff = Utils::solveQP(A.transpose() * A, -1*f, -1*A_constraint, -1*b_constraint);
        } else {
            Eigen::MatrixXd A_last(2,A.cols());
            A_last << A.row(k-1), A.row(2*k-1);

            Eigen::VectorXd b_last(2);
            b_last << 
            x(k-1) - Axx(k-1,n-1) * V(0) - Axx(k-1,n-2) * Acc(0)*0.5 - Axx(k-1,n) * x(0),
            y(k-1) - Axx(k-1,n-1) * V(1) - Axx(k-1,n-2) * Acc(1)*0.5 - Axx(k-1,n) * y(0);

            A_equality << A_constraint.block(k-1,0,1,A_constraint.cols()),
            A_constraint.block(2*k-1,0,1,A_constraint.cols()),
            A_constraint.block(5*k -1,0,1,A_constraint.cols()),
            A_constraint.block(6*k -1,0,1,A_constraint.cols()),
            A_last;

            b_equality << VectorXd::Zero(4), b_last;

            //if (i==0){
            //cout << A_equality << std::endl;
            //cout << b_equality << std::endl;
            //}
            /*
            Eigen::MatrixXd A_constraint_new(A_constraint.rows()-8,A_constraint.cols());
            Eigen::VectorXd b_constraint_new(b_constraint.size()-8);
            A_constraint_new << A_constraint.block(0,0,k-1,A_constraint.cols()),
            A_constraint.block(k,0,k-1,A_constraint.cols()),
            A_constraint.block(k*2,0,k-1,A_constraint.cols()),
            A_constraint.block(k*3,0,k-1,A_constraint.cols()),
            A_constraint.block(k*4,0,k-1,A_constraint.cols()),
            A_constraint.block(k*5,0,k-1,A_constraint.cols()),
            A_constraint.block(k*6,0,k-1,A_constraint.cols()),
            A_constraint.block(k*7,0,k-1,A_constraint.cols());
            b_constraint_new << b_constraint.segment(0,k-1),
            b_constraint.segment(-1+k,k-1),
            b_constraint.segment(-1+k*2,k-1),
            b_constraint.segment(-1+k*3,k-1),
            b_constraint.segment(-1+k*4,k-1),
            b_constraint.segment(-1+k*5,k-1),
            b_constraint.segment(-1+k*6,k-1),
            b_constraint.segment(-1+k*7,k-1);
*/
            coeff = Utils::solveQP(A.transpose() * A, -f, -1*A_constraint, -1*b_constraint, A_equality, b_equality);
        }

	
	if (coeff.size() >0){		    	
	VectorXd px_(n + 1), py_(n  + 1);
	
	// Assign values to px_
	px_.head(n - 2) = coeff.head(n - 2);
	px_(n - 2) = 0.5 * Acc(0);
	px_(n - 1) = V(0);
	px_(n) = x(0);

	// Assign values to py_
	py_.head(n-2) = coeff.tail(n - 2);
	py_(n-2) = 0.5 * Acc(1);
	py_(n - 1 ) = V(1);
	py_(n) = y(0);

	VectorXd X_ = VectorXd::Zero(2*k);
	for (int j=0; j<=n; j++)
	{
	VectorXd pow_s = (t * s).array().pow(n - j);
	X_.head(k) += pow_s*px_(j);
	X_.tail(k) += pow_s*py_(j);
	}

	VectorXd X_ref(2*k);
	X_ref << x, y;
	VectorXd diff = X_  - X_ref;
	
	double val = t*weights(0) + diff.norm()*weights(1);
	
	if (val < val_min) {
	    val_min = val;
	    val_t = t;	

	    pp << px_.transpose(), py_.transpose();		

	    X = X_;
	}
	}
	
      

    }
      t_final = val_t;
      VectorXd px = pp.topRows(1).transpose();
      VectorXd py = pp.bottomRows(1).transpose();

      VectorXd pdx (n), pdy(n), pddx(n-1), pddy(n-1);
      for (int i=0; i < n; i++)
      {
      	pdx(i) = (n-i) * px(i);
      	pdy(i) = (n-i) * py(i);
      	if (i < n-1){
      	pddx(i) = (n-i-1) * pdx(i);
      	pddy(i) = (n-i-1) * pdy(i);}
      }
      	pd << pdx.transpose() , pdy.transpose();
      	pdd << pddx.transpose() , pddy.transpose();
      //MatrixXd Xd(2,k), Xdd(2,k);
      Xd = MatrixXd::Zero(2,k);
      Xdd = MatrixXd::Zero(2,k);
      
      Xd << pdx(n-1)*VectorXd::Ones(k).transpose(),
      pdy(n-1)*VectorXd::Ones(k).transpose();
      Xdd << pddx(n-2)*VectorXd::Ones(k).transpose(),
      pddy(n-2)*VectorXd::Ones(k).transpose();
      
	
      s = s*val_t;
      for (int i = n-2; i>=0 ; i--)
      {
      Xd.block(0,0,1,k) += pdx(i) * s.transpose();
      Xd.block(1,0,1,k) += pdy(i) * s.transpose();
      if (i>0){
      Xdd.block(0,0,1,k) += pddx(i-1) * s.transpose();
      Xdd.block(1,0,1,k) += pddy(i-1) * s.transpose();}
      s = s*val_t;
      }
      //std::cout << Xd << std::endl;
      //std::cout << Xdd << std::endl;
    return X;
}
