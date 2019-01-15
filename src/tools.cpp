#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    //1.check for valid inputs,
    //vector size should not be zero
    //vector size should equal to grounf truth vector size
    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
        cout << "Invalid estimations or ground-truth information" << endl;
        return rmse;
    }

    //2.accumulate squared residuals 
    for (unsigned int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication for processing
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //3.calculating mean
    rmse = rmse / estimations.size();

    //4.sqrt calulating
    rmse = sqrt(rmse.array());

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{

    MatrixXd Hj(3, 4);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //avoid repeated calculations
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = c2 * c1;

    //check division by zero
    if (fabs(c1) < 0.0001) {
        cout << "Error! - eivision by zero were happened!" << endl;
        return Hj;
    }

    Hj << px / c2, py / c2, 0, 0, -py / c1, px / c1, 0, 0, py*(vx * py - vy * px) / c3, px*(vy * px - vx * py) / c3, px / c2, py / c2;

    return Hj;
}

VectorXd Tools::PolarToCartesian(const VectorXd& polar)
{

    VectorXd cartesian = VectorXd(4);

    float rho = polar(0);
    float phi = polar(1);
    float rhodot = polar(2);

    float px = rho * cos(phi);
    float py = rho * sin(phi);
    float vx = rhodot * cos(phi);
    float vy = rhodot * sin(phi);

    cartesian << px, py, vx, vy;
    return cartesian;
}

VectorXd Tools::CartesianToPolar(const VectorXd& cartesian)
{
    VectorXd polar = VectorXd(3);
    float thrsh_ = 0.0001;
    float px = cartesian(0);
    float py = cartesian(1);
    float vx = cartesian(2);
    float vy = cartesian(3);

    float rho = sqrt(px * px + py * py);
    float phi = 0;
    float rhodot = 0;

    //handle undefined value for phi, if px and py are zero
    if (px != 0 && py != 0)
        phi = atan2(py, px);


    //avoid Divide by zero when radian distance from origin is small
    if (rho > thrsh_) {
        rhodot = (px * vx + py * vy) / rho;
    }

    polar << rho, phi, rhodot;

    return polar;
}
