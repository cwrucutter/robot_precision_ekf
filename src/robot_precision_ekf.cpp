/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2013, EJ Kreinar, Case Western Reserve University
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <robot_precision_ekf/robot_precision_ekf.h>

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;

// constructor
RobotPrecisionEKF::RobotPrecisionEKF(double timestep):
  prior_(NULL),
  filter_(NULL),
  filter_initialized_(false),
  gps_initialized_(false),
  odom_initialized_(false),
  imu_initialized_(false),
  dt_(timestep)
{
  /****************************
   * NonLinear system model      *
   ***************************/

  // create gaussian
  ColumnVector sys_noise_Mu(STATE_SIZE);
  sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
  sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
  sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;
  sys_noise_Mu(4) = MU_SYSTEM_NOISE_VEL;
  sys_noise_Mu(5) = MU_SYSTEM_NOISE_OMG;

  SymmetricMatrix sys_Q(STATE_SIZE);
  sys_Q = 0.0;
  sys_Q(1,1) = SIGMA_SYSTEM_NOISE_X*dt_; sys_Q(1,2) = 0.0; sys_Q(1,3) = 0.0; sys_Q(1,4) = 0.0; sys_Q(1,5) = 0.0;
  sys_Q(2,1) = 0.0; sys_Q(2,2) = SIGMA_SYSTEM_NOISE_Y*dt_; sys_Q(2,3) = 0.0; sys_Q(2,4) = 0.0; sys_Q(2,5) = 0.0;
  sys_Q(3,1) = 0.0; sys_Q(3,2) = 0.0; sys_Q(3,3) = SIGMA_SYSTEM_NOISE_THETA*dt_; sys_Q(3,4) = 0.0; sys_Q(3,5) = 0.0;
  sys_Q(4,1) = 0.0; sys_Q(4,2) = 0.0; sys_Q(4,3) = 0.0; sys_Q(4,4) = SIGMA_SYSTEM_NOISE_VEL*dt_; sys_Q(4,5) = 0.0;
  sys_Q(5,1) = 0.0; sys_Q(5,2) = 0.0; sys_Q(5,3) = 0.0; sys_Q(5,4) = 0.0; sys_Q(5,5) = SIGMA_SYSTEM_NOISE_OMG*dt_;

  Gaussian system_Uncertainty(sys_noise_Mu, sys_Q);

  // create the model
  sys_pdf_ = new NonLinearAnalyticConditionalGaussianRobot(system_Uncertainty, dt_);
  sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

  /*********************************
   * Initialise measurement model *
   ********************************/

  // XY MEASUREMENT at Arbitrary relationship to origin (nonlinear)
  // y = [xgps;  = [x + xarm*cos(tht) - yarm*sin(tht); 
  //      ygps]     y + xarm*sin(tht) + yarm*cos(tht)]
  // Construct the measurement noise
  ColumnVector meas_noise_Mu_gps(GPS_MEAS_SIZE);
  meas_noise_Mu_gps(1) = GPS_MU_MEAS_NOISE_X;
  meas_noise_Mu_gps(2) = GPS_MU_MEAS_NOISE_Y;
  SymmetricMatrix meas_R_gps(GPS_MEAS_SIZE);
  meas_R_gps(1,1) = GPS_SIGMA_MEAS_NOISE_X; meas_R_gps(1,2) = 0.0;
  meas_R_gps(2,1) = 0.0; meas_R_gps(2,2) = GPS_SIGMA_MEAS_NOISE_Y;
  Gaussian measurement_Uncertainty_gps(meas_noise_Mu_gps, meas_R_gps);

  // create the measurement model
  gps_meas_pdf_   = new NonLinearAnalyticConditionalGaussianGPSMeasurement(measurement_Uncertainty_gps);
  gps_meas_model_ = new AnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);
  
  
  // ODOMETRY MEASUREMENT MODEL (linear)
  // y = [vR; = [v+b/2; 
  //      vL]    v-b/2]
  Matrix H_odom(ODOM_MEAS_SIZE,STATE_SIZE);
  H_odom = 0.0; //TODO: Replace the hardcoded track width with the track with from odometry
  H_odom(1,1) = 0.0; H_odom(1,2) = 0.0; H_odom(1,3) = 0.0; H_odom(1,4) = 1.0; H_odom(1,5) = ODOM_TRACK/2;
  H_odom(2,1) = 0.0; H_odom(2,2) = 0.0; H_odom(2,3) = 0.0; H_odom(2,4) = 1.0; H_odom(2,5) = -ODOM_TRACK/2;
  
  // Construct the measurement noise
  ColumnVector meas_noise_Mu_odom(ODOM_MEAS_SIZE);
  meas_noise_Mu_odom(1) = ODOM_MU_MEAS_NOISE_X;
  meas_noise_Mu_odom(2) = ODOM_MU_MEAS_NOISE_Y;
  SymmetricMatrix meas_R_odom(ODOM_MEAS_SIZE);
  meas_R_odom(1,1) = ODOM_SIGMA_MEAS_NOISE_R; meas_R_odom(1,2) = 0.0;
  meas_R_odom(2,1) = 0.0; meas_R_odom(2,2) = ODOM_SIGMA_MEAS_NOISE_L;
  Gaussian measurement_Uncertainty_odom(meas_noise_Mu_odom, meas_R_odom);

  // create the measurement model
  odom_meas_pdf_   = new LinearAnalyticConditionalGaussian(H_odom, measurement_Uncertainty_odom);
  odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);


  /****************************
   * Linear prior DENSITY     *
   ***************************/
   // Continuous Gaussian prior (for Kalman filters)
  ColumnVector prior_Mu(STATE_SIZE);
  prior_Mu(1) = PRIOR_MU_X;
  prior_Mu(2) = PRIOR_MU_Y;
  prior_Mu(3) = PRIOR_MU_THETA;
  prior_Mu(4) = PRIOR_MU_VEL;
  prior_Mu(5) = PRIOR_MU_OMG;
  SymmetricMatrix prior_Cov(STATE_SIZE);
  prior_Cov(1,1) = PRIOR_COV_X; prior_Cov(1,2) = 0.0; prior_Cov(1,3) = 0.0; prior_Cov(1,4) = 0.0; prior_Cov(1,5) = 0.0;
  prior_Cov(2,1) = 0.0; prior_Cov(2,2) = PRIOR_COV_Y; prior_Cov(2,3) = 0.0; prior_Cov(2,4) = 0.0; prior_Cov(2,5) = 0.0;
  prior_Cov(3,1) = 0.0; prior_Cov(3,2) = 0.0; prior_Cov(3,3) = PRIOR_COV_THETA; prior_Cov(3,4) = 0.0; prior_Cov(3,5) = 0.0;
  prior_Cov(4,1) = 0.0; prior_Cov(4,2) = 0.0; prior_Cov(4,3) = 0.0; prior_Cov(4,4) = PRIOR_COV_VEL; prior_Cov(4,5) = 0.0;
  prior_Cov(5,1) = 0.0; prior_Cov(5,2) = 0.0; prior_Cov(5,3) = 0.0; prior_Cov(5,4) = 0.0; prior_Cov(5,5) = PRIOR_COV_OMG;
  prior_  = new Gaussian(prior_Mu,prior_Cov);

  /******************************
   * Construction of the Filter *
   ******************************/
  filter_ = new ExtendedKalmanFilter(prior_);

};


// destructor
RobotPrecisionEKF::~RobotPrecisionEKF(){
  if (filter_) delete filter_;
  if (prior_)  delete prior_;
  delete odom_meas_model_;
  delete odom_meas_pdf_;
  //delete imu_meas_model_;
  //delete imu_meas_pdf_;
  delete gps_meas_model_;
  delete gps_meas_pdf_;
  delete sys_pdf_;
  delete sys_model_;
};


