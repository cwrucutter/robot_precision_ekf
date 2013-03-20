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

typedef NonLinearAnalyticConditionalGaussianRobot  RobotPdf5State;
typedef NonLinearAnalyticConditionalGaussian3State RobotPdf3State;

// constructor
RobotPrecisionEKF::RobotPrecisionEKF(FilterType type, double timestep, ColumnVector sysNoise):
  prior_(NULL),
  filter_(NULL),
  filter_initialized_(false),
  odom_initialized_(false),
  imu_initialized_(false),
  gps_initialized_(false),
  dt_(timestep),
  new_input_(false)
{
 
  /*********************
   * SET FILTER TYPE   *
   ********************/
   
  filter_type_ = type;
  if (filter_type_ == EKF_5STATE)
    state_size_ = 5;
  else if (filter_type_ == RobotPrecisionEKF::EKF_3STATE)
    state_size_ = 3;
  else if (filter_type_ == RobotPrecisionEKF::EKF_7STATE_VERR)
    state_size_ = 7;
  else
  {
    ROS_WARN("Unknown Filter type. Cannot Initialize EKF");
    return;
  }
  
  /****************************
   * NonLinear system model   *
   ***************************/

  // Initialize using the provided system noise!
  filter_initialized_ = initSystem(sysNoise); // Initializes the system_pdf_, system_model_ and prior_
  
  /*********************************
   * Initialize measurement models *
   ********************************/
   
  // Do NOT initialize measurement models here.
  // Nodes implementing this class must explicitly call the 
  //   functions: initMeasEnc, initMeasGPS, and initMeasIMU
  //   in order to initialize the measurement models with 
  //   desired noise values
  
  /******************************
   * Construction of the Filter *
   ******************************/
  if (filter_initialized_)
    filter_ = new ExtendedKalmanFilter(prior_);

};

bool RobotPrecisionEKF::initSystem(ColumnVector noiseIn)
{
  // TODO: Verify that noiseIn is the correct size
  if (noiseIn.size() < state_size_)
  {
    ROS_WARN("RobotPrecisionEKF::initSystem failed because \
        supplied system noise does not include enough terms");
    return false;
  }
  
  ColumnVector sys_noise_Mu(state_size_);
  SymmetricMatrix sys_Q(state_size_);
  sys_Q = 0.0;
  
  ColumnVector prior_Mu(state_size_);
  SymmetricMatrix prior_Cov(state_size_);
  prior_Cov = 0.0;
  
  Gaussian system_Uncertainty;
  
  // System depends on the filter type desired
  switch (filter_type_)
  {
    // TODO: Build a new "NonLinearAnalyticConditionalGaussianRobot" for the 3 state and 5 state case
    // TODO: Implement Iterated EKF or UKF??
    
    case RobotPrecisionEKF::EKF_5STATE:
      sys_noise_Mu(1) = MU_SYSTEM_NOISE_X; sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y; sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;
      sys_noise_Mu(4) = MU_SYSTEM_NOISE_VEL; sys_noise_Mu(5) = MU_SYSTEM_NOISE_OMG;
      
      sys_Q(1,1) = noiseIn(1)*dt_; sys_Q(2,2) = noiseIn(2)*dt_; sys_Q(3,3) = noiseIn(3)*dt_;
      sys_Q(4,4) = noiseIn(4)*dt_; sys_Q(5,5) = noiseIn(5)*dt_;
      
      // Create Gaussian
      system_Uncertainty.ExpectedValueSet(sys_noise_Mu);
      system_Uncertainty.CovarianceSet(sys_Q);

      // create the model
      sys_pdf_ = new NonLinearAnalyticConditionalGaussianRobot(system_Uncertainty, dt_);
      sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);
      
      // Continuous Gaussian prior (for Kalman filters)
      prior_Mu(1) = PRIOR_MU_X; // This is just set to something arbitrary because the 
      prior_Mu(2) = PRIOR_MU_Y; // filter shold be able to figure it all out. Or something
      prior_Mu(3) = PRIOR_MU_THETA; prior_Mu(4) = PRIOR_MU_VEL; prior_Mu(5) = PRIOR_MU_OMG;
      prior_Cov(1,1) = PRIOR_COV_X; prior_Cov(2,2) = PRIOR_COV_Y; prior_Cov(3,3) = PRIOR_COV_THETA;
      prior_Cov(4,4) = PRIOR_COV_VEL; prior_Cov(5,5) = PRIOR_COV_OMG;
      prior_  = new Gaussian(prior_Mu,prior_Cov);
      return true;
      
    case RobotPrecisionEKF::EKF_3STATE:
      sys_noise_Mu(1) = MU_SYSTEM_NOISE_X; sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y; sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;
      sys_Q(1,1) = noiseIn(1)*dt_; sys_Q(2,2) = noiseIn(2)*dt_; sys_Q(3,3) = noiseIn(3)*dt_;

      // Create Gaussian
      system_Uncertainty.ExpectedValueSet(sys_noise_Mu);
      system_Uncertainty.CovarianceSet(sys_Q);

      // create the model
      sys_pdf_ = new NonLinearAnalyticConditionalGaussian3State(system_Uncertainty, dt_);
      sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);
      
      // Continuous Gaussian prior (for Kalman filters)
      prior_Mu(1) = PRIOR_MU_X; // This is just set to something arbitrary because the 
      prior_Mu(2) = PRIOR_MU_Y; // filter shold be able to figure it all out. Or something
      prior_Mu(3) = PRIOR_MU_THETA;
      prior_Cov(1,1) = PRIOR_COV_X; prior_Cov(2,2) = PRIOR_COV_Y; prior_Cov(3,3) = PRIOR_COV_THETA;
      prior_  = new Gaussian(prior_Mu,prior_Cov);
      return true;
      
    case RobotPrecisionEKF::EKF_7STATE_VERR:
      sys_noise_Mu(1) = MU_SYSTEM_NOISE_X; sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y; sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;
      sys_noise_Mu(4) = MU_SYSTEM_NOISE_VEL; sys_noise_Mu(5) = MU_SYSTEM_NOISE_OMG;
      sys_noise_Mu(6) = MU_SYSTEM_NOISE_VRERR; sys_noise_Mu(7) = MU_SYSTEM_NOISE_VLERR;
      
      sys_Q(1,1) = noiseIn(1)*dt_; sys_Q(2,2) = noiseIn(2)*dt_; sys_Q(3,3) = noiseIn(3)*dt_;
      sys_Q(4,4) = noiseIn(4)*dt_; sys_Q(5,5) = noiseIn(5)*dt_;
      sys_Q(6,6) = noiseIn(6)*dt_; sys_Q(7,7) = noiseIn(7)*dt_;
      
      // Create Gaussian
      system_Uncertainty.ExpectedValueSet(sys_noise_Mu);
      system_Uncertainty.CovarianceSet(sys_Q);

      // create the model
      sys_pdf_ = new NonLinearAnalyticConditionalGaussianRobotVerr(system_Uncertainty, dt_);
      sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);
      
      // Continuous Gaussian prior (for Kalman filters)
      prior_Mu(1) = PRIOR_MU_X; // This is just set to something arbitrary because the 
      prior_Mu(2) = PRIOR_MU_Y; // filter shold be able to figure it all out. Or something
      prior_Mu(3) = PRIOR_MU_THETA; prior_Mu(4) = PRIOR_MU_VEL; prior_Mu(5) = PRIOR_MU_OMG;
      prior_Mu(6) = PRIOR_MU_VRERR; prior_Mu(7) = PRIOR_MU_VLERR;
      prior_Cov(1,1) = PRIOR_COV_X; prior_Cov(2,2) = PRIOR_COV_Y; prior_Cov(3,3) = PRIOR_COV_THETA;
      prior_Cov(4,4) = PRIOR_COV_VEL; prior_Cov(5,5) = PRIOR_COV_OMG;
      prior_Cov(6,6) = PRIOR_COV_VRERR; prior_Cov(7,7) = PRIOR_COV_VLERR;
      prior_  = new Gaussian(prior_Mu,prior_Cov);
      return true;
      
    default:
      return false;
  }
  
  return false;
}

bool RobotPrecisionEKF::initMeasOdom(double alpha, double epsilon)
{  
  odom_alpha_ = alpha;
  odom_eps_ = epsilon;
  
  Matrix H_odom(ODOM_MEAS_SIZE,state_size_);
  H_odom = 0.0;
      
  ColumnVector meas_noise_Mu_odom(ODOM_MEAS_SIZE);
  SymmetricMatrix meas_R_odom(ODOM_MEAS_SIZE);
  meas_R_odom = 0.0;
  
  Gaussian measurement_Uncertainty_odom;
  
  // Measurement depends on the filter type desired
  switch (filter_type_)
  {
    case RobotPrecisionEKF::EKF_5STATE:
      // ODOMETRY MEASUREMENT MODEL (linear)
      // y = [vR; = [v+b/2; 
      //      vL]    v-b/2]
    
      //TODO: Replace the hardcoded track width with the track from odometry parameter
      H_odom(1,4) = 1.0; H_odom(1,5) = ODOM_TRACK/2;
      H_odom(2,4) = 1.0; H_odom(2,5) = -ODOM_TRACK/2;
      
      // Construct the measurement noise
      meas_noise_Mu_odom(1) = ODOM_MU_MEAS_NOISE_X;
      meas_noise_Mu_odom(2) = ODOM_MU_MEAS_NOISE_Y;
      meas_R_odom(1,1) = odom_eps_;
      meas_R_odom(2,2) = odom_eps_;
      
      measurement_Uncertainty_odom.ExpectedValueSet(meas_noise_Mu_odom);
      measurement_Uncertainty_odom.CovarianceSet(meas_R_odom);

      // create the measurement model
      odom_meas_pdf_   = new LinearAnalyticConditionalGaussian(H_odom, measurement_Uncertainty_odom);
      odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);
      odom_initialized_ = true;
      return true;
      
    case RobotPrecisionEKF::EKF_3STATE:
      // Odometry is handled as an input! Not a measurement
      try
      {
        dynamic_cast<NonLinearAnalyticConditionalGaussian3State *>(sys_pdf_)->setOdomNoise(odom_alpha_,odom_eps_);
      }
      catch (std::bad_cast err)
      {
        ROS_ERROR("Odometry Initialization Failed for 3-State EKF");
        return false;
      }
      return true;
      
    case RobotPrecisionEKF::EKF_7STATE_VERR:
      // ODOMETRY MEASUREMENT MODEL (linear)
      // y = [vR + vRerr; = [v+w*b/2+vRerr; 
      //      vL + vLerr]    v-w*b/2+vLerr]
    
      //TODO: Replace the hardcoded track width with the track from odometry parameter
      H_odom(1,4) = 1.0; H_odom(1,5) =  ODOM_TRACK/2; //H_odom(1,6) = 0.0;
      H_odom(2,4) = 1.0; H_odom(2,5) = -ODOM_TRACK/2; //H_odom(1,7) = 0.0;
      
      // Construct the measurement noise
      meas_noise_Mu_odom(1) = ODOM_MU_MEAS_NOISE_X;
      meas_noise_Mu_odom(2) = ODOM_MU_MEAS_NOISE_Y;
      meas_R_odom(1,1) = odom_eps_;
      meas_R_odom(2,2) = odom_eps_;
      
      measurement_Uncertainty_odom.ExpectedValueSet(meas_noise_Mu_odom);
      measurement_Uncertainty_odom.CovarianceSet(meas_R_odom);

      // create the measurement model
      odom_meas_pdf_   = new LinearAnalyticConditionalGaussian(H_odom, measurement_Uncertainty_odom);
      odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);
      odom_initialized_ = true;
      return true;
      
    default:
      return false;
  }
  
  return false;
}

bool RobotPrecisionEKF::initMeasGPS(ColumnVector noiseIn)
{
  if (noiseIn.size() != GPS_MEAS_SIZE)
  {
    ROS_WARN("GPS initialization failed because supplied \
        measurement noise does not match GPS measurement size");
    return false;
  }
  
  ColumnVector meas_noise_Mu_gps(GPS_MEAS_SIZE);
  SymmetricMatrix meas_R_gps(GPS_MEAS_SIZE);
  meas_R_gps = 0.0;
  
  Gaussian measurement_Uncertainty_gps;
  
  // Measurement depends on the filter type desired
  switch (filter_type_)
  {
    case RobotPrecisionEKF::EKF_7STATE_VERR:
    case RobotPrecisionEKF::EKF_5STATE:
    case RobotPrecisionEKF::EKF_3STATE:
      // XY MEASUREMENT at Arbitrary relationship to origin (nonlinear)
      // y = [xgps;  = [x + xarm*cos(tht) - yarm*sin(tht); 
      //      ygps]     y + xarm*sin(tht) + yarm*cos(tht)]
      // Construct the measurement noise
      meas_noise_Mu_gps(1) = GPS_MU_MEAS_NOISE_X;
      meas_noise_Mu_gps(2) = GPS_MU_MEAS_NOISE_Y;
      meas_R_gps(1,1) = noiseIn(1);
      meas_R_gps(2,2) = noiseIn(2);
      measurement_Uncertainty_gps.ExpectedValueSet(meas_noise_Mu_gps);
      measurement_Uncertainty_gps.CovarianceSet(meas_R_gps);

      // create the measurement model
      gps_meas_pdf_   = new NonLinearAnalyticConditionalGaussianGPSMeasurement(measurement_Uncertainty_gps, state_size_);
      gps_meas_model_ = new AnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);
      gps_initialized_ = true;
      return true;
      
    default:
      return false;
  }
  
  return false;
}

bool RobotPrecisionEKF::initMeasIMU(ColumnVector noiseIn)
{
  // TODO: Implement IMU measurement
  return false;
}

void RobotPrecisionEKF::systemUpdate()
{
 
  // System update
  switch (filter_type_)
  {
    case EKF_5STATE:
    case EKF_7STATE_VERR:
      filter_->Update(sys_model_);
      break;
      
    case EKF_3STATE:
      if (!new_input_)
      {
        ColumnVector temp(2);
        temp(1) = 0.0; temp(2) = 0.0;
        inputs_ = temp;
      }
      filter_->Update(sys_model_,inputs_);
      break;
      
    default:
      // Do nothing..?
      break;
  }
  // Store posterior
  posterior_ = filter_->PostGet();
}

void RobotPrecisionEKF::measurementUpdateGPS(double x, double y)
{
  // Prepare measurement
  ColumnVector gps(GPS_MEAS_SIZE);
  gps(1) = x;
  gps(2) = y;
  
  // Update
  filter_->Update(gps_meas_model_, gps);
  
  // Store posterior
  posterior_ = filter_->PostGet();
}

void RobotPrecisionEKF::measurementUpdateOdom(double vR, double vL)
{
  //Prepare measurement
  ColumnVector odom(ODOM_MEAS_SIZE);
  odom(1) = vR;
  odom(2) = vL;
  MatrixWrapper::SymmetricMatrix odomNoise(ODOM_MEAS_SIZE);
  
  switch (filter_type_)
  {
    case EKF_5STATE:
    case EKF_7STATE_VERR:
      // Dynamic odometry noise, based on each wheel vel
      odomNoise(1,1) = odom_alpha_ * fabs(vR) + odom_eps_;   //  = vR^2 * alpha  +  epsilon
      odomNoise(2,2) = odom_alpha_ * fabs(vL) + odom_eps_;   //  = vL^2 * alpha  +  epsilon
      odom_meas_pdf_->AdditiveNoiseSigmaSet(odomNoise);
      odom_meas_model_->MeasurementPdfSet(odom_meas_pdf_); // TODO: Do I really need to Re-set the MeasurementPdf? Or can I just modify that?
      
      // Update
      filter_->Update(odom_meas_model_,odom);
      
      // Store posterior
      posterior_ = filter_->PostGet();
      break;
      
    case EKF_3STATE:
      // Store the odometry to use during the next SYSTEM update
      inputs_ = odom;
      new_input_ = true;
      break;
      
    default:
      // Do nothing..?
      break;
  }
}


MatrixWrapper::ColumnVector RobotPrecisionEKF::getMean()
{
  return posterior_->ExpectedValueGet();
}
  
MatrixWrapper::SymmetricMatrix RobotPrecisionEKF::getCovariance()
{
  return posterior_->CovarianceGet();
}

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


