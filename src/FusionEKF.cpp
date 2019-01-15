#include "FusionEKF.h"
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_lidar_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_lidar_ = MatrixXd(2, 4);

    // measurement covariance matrix - laser
    R_lidar_ << 0.0225, 0, 0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

    // measurement matrix for laser
    H_lidar_ << 1, 0, 0, 0, 0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          */

        // 1st measurement
        ekf_.x_ = VectorXd(4);
        previous_timestamp_ = measurement_pack.timestamp_;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /*
             * Convert radar from polar to cartesian coordinates and initialize state.
             */
            VectorXd polar = VectorXd(3);
            polar << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), measurement_pack.raw_measurements_(2);
            VectorXd cartesian = tools.PolarToCartesian(polar);
            ekf_.x_ = cartesian;
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR) {
            /*
             *Initialize state.
             */
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;

            //state covariance matrix P_
            ekf_.P_ = MatrixXd(4, 4);
            ekf_.P_ <<  1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

            //the initial transition matrix F_
            ekf_.F_ = MatrixXd(4, 4);
            ekf_.F_ <<  1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
        }

        is_initialized_ = true;

        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for Q matrix.
     */
    //set the acceleration noise components
    noise_ax = 9;
    noise_ay = 9;

    //dt: expresse in seconds
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //Modify the F_ matrix, to let the time be integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //set the process noise covariance matrix Q_
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0, 0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0, 0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        ekf_.Update_Radar(measurement_pack.raw_measurements_);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LIDAR) {
        // Lidar updates
        ekf_.R_ = R_lidar_;
        ekf_.H_ = H_lidar_;
        ekf_.Update_Lidar(measurement_pack.raw_measurements_);
    }
}
