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

#ifndef __ROBOT_PRECISION_EKF__
#define __ROBOT_PRECISION_EKF__

// bayesian filtering
#include <wrappers/matrix/matrix_wrapper.h>

#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussian_robot.h"
#include "nonlinearanalyticconditionalgaussian_gpsmeasurement.h"

#include "ekf_const.h"

// TF
#include <tf/tf.h>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// log files
#include <fstream>

class RobotPrecisionEKF
{
public:

  enum FilterType {
    EKF_5STATE = 0,
    EKF_3STATE_INPUTS
  };

  /// constructor
  RobotPrecisionEKF(FilterType type, double timestep, MatrixWrapper::ColumnVector sysNoise);
  
  
  bool initSystem(MatrixWrapper::ColumnVector noiseIn);
  
  // TODO: Abstract the measurements so RobotPrecisionEKF simply calls the initialization
  //       and other calls for an arbitrary number of sensors which could be defined as
  //       derived classes/ plugins in the future (??)
  bool initMeasOdom(double alpha, double epsilon);
  
  bool initMeasGPS(MatrixWrapper::ColumnVector noiseIn);
  
  bool initMeasIMU(MatrixWrapper::ColumnVector noiseIn);
  
  
  void systemUpdate();
  
  void measurementUpdateGPS(double x, double y);
  
  void measurementUpdateOdom(double vR, double vL);
  
  MatrixWrapper::ColumnVector getMean();
  
  MatrixWrapper::SymmetricMatrix getCovariance();

  /// destructor
  virtual ~RobotPrecisionEKF();

private:
  FilterType filter_type_;
  int state_size_;

  // pdf / model / filter
  BFL::NonLinearAnalyticConditionalGaussianRobot*          sys_pdf_;
  BFL::AnalyticSystemModelGaussianUncertainty*             sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianGPSMeasurement* gps_meas_pdf_;
  BFL::AnalyticMeasurementModelGaussianUncertainty*        gps_meas_model_;
  BFL::LinearAnalyticConditionalGaussian*                  odom_meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty*  odom_meas_model_;
  //BFL::LinearAnalyticConditionalGaussian*                 imu_meas_pdf_;
  //BFL::LinearAnalyticMeasurementModelGaussianUncertainty* imu_meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  MatrixWrapper::SymmetricMatrix                          sys_covariance_, gps_covariance_, odom_covariance_, imu_covariance_;
  MatrixWrapper::ColumnVector                             inputs_;
  
  BFL::Pdf<MatrixWrapper::ColumnVector> * posterior_;

  // vars
  bool filter_initialized_, odom_initialized_, imu_initialized_, gps_initialized_;
  double dt_;
  double odom_alpha_, odom_eps_;
  bool new_input_;
  /*MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
  tf::Transform filter_estimate_old_;
  tf::StampedTransform odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, gps_meas_, gps_meas_old_;
  ros::Time filter_time_old_;*/

  // tf transformer
  tf::Transformer transformer_;

}; // class


#endif
