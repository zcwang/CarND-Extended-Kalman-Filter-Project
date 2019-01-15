#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools
{
public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
     * Calculate Jacobians.
     */
    MatrixXd CalculateJacobian(const VectorXd& x_state);

    /**
     * Convert polar coordinates to cartesian coordinates.
     */
    VectorXd PolarToCartesian(const VectorXd& polar);

    /**
     * Convert cartesian coordinates to polar coordinates.
     */
    VectorXd CartesianToPolar(const VectorXd& cartesian);

};

#endif /* TOOLS_H_ */
