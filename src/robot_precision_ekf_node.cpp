/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, EJ Kreinar, Case Western Reserve University
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

// Based loosely on the robot_pose_ekf program by Wim Meeussen
//  and Willow Garage

#include <robot_precision_ekf/robot_precision_ekf_node.h>

#include <iostream>

using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


static const double EPS = 1e-5;

// constructor
RobotPrecisionEKFNode::RobotPrecisionEKFNode()
{
  ros::NodeHandle nh_private("~");
  ros::NodeHandle nh;
  
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  // General Parameters
  nh_private.param("global_frame_id", global_frame_id_, std::string("map"));
  nh_private.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh_private.param("sensor_timeout", timeout_, 1.0);
  
  // Filter parameters
  nh_private.param("odom_used", odom_used_, true);
  nh_private.param("imu_used",  imu_used_, true);
  nh_private.param("gps_used",   gps_used_, true);
  double freq;
  nh_private.param("freq", freq, 10.0);
  double tmp_tol;
  nh_private.param("transform_tolerance", tmp_tol, 0.1);
  
  // Noise parameters
  nh_private.param("sigma_sys_x",  sigma_sys_x_, 0.01);
  nh_private.param("sigma_sys_y",  sigma_sys_y_, 0.01);
  nh_private.param("sigma_sys_tht",  sigma_sys_tht_, 0.05);
  nh_private.param("sigma_sys_vel",  sigma_sys_vel_, 0.5);
  nh_private.param("sigma_sys_omg",  sigma_sys_omg_, 0.5);
  nh_private.param("sigma_meas_gps_x",  sigma_meas_gps_x_, 0.05);
  nh_private.param("sigma_meas_gps_y",  sigma_meas_gps_x_, 0.05);
  nh_private.param("sigma_meas_odom_vR",  sigma_meas_odom_vR_, 0.05);
  nh_private.param("sigma_meas_odom_vL",  sigma_meas_odom_vL_, 0.05);
  
  // Node parameters
  nh_private.param("debug",   debug_, false);
  nh_private.param("self_diagnose",  self_diagnose_, false);

  ekf_filter_ = new RobotPrecisionEKF(1.0/max(freq,1.0));

  timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &RobotPrecisionEKFNode::spin, this);
  transform_tolerance_.fromSec(tmp_tol);

  // advertise our estimation
  pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 2);

  // initialize
  filter_stamp_ = Time::now();

  // subscribe to odom messages
  if (odom_used_){
    ROS_INFO("Odom sensor can be used");
    odom_sub_ = nh.subscribe("odom", 10, &RobotPrecisionEKFNode::odomCallback, this);
  }
  else ROS_INFO("Odom sensor will NOT be used");

  // subscribe to imu messages
  if (imu_used_){
    ROS_INFO("Imu sensor can be used");
    imu_sub_ = nh.subscribe("imu/data", 10,  &RobotPrecisionEKFNode::imuCallback, this);
  }
  else ROS_INFO("Imu sensor will NOT be used");

  // subscribe to vo messages
  if (gps_used_){
    ROS_INFO("VO sensor can be used");
    gps_sub_ = nh.subscribe("gps_pose", 10, &RobotPrecisionEKFNode::gpsCallback, this);
  }
  else ROS_INFO("VO sensor will NOT be used");

  // publish state service
  //state_srv_ = nh_private.advertiseService("get_status", &RobotPrecisionEKFNode::getStatus, this);

  if (debug_){
    // open files for debugging
    odom_file_.open("/tmp/odom_file.txt");
    imu_file_.open("/tmp/imu_file.txt");
    gps_file_.open("/tmp/gps_file.txt");
    corr_file_.open("/tmp/corr_file.txt");
    time_file_.open("/tmp/time_file.txt");
    extra_file_.open("/tmp/extra_file.txt");
  }
};




// destructor
RobotPrecisionEKFNode::~RobotPrecisionEKFNode(){

  if (debug_){
    // close files for debugging
    odom_file_.close();
    imu_file_.close();
    gps_file_.close();
    corr_file_.close();
    time_file_.close();
    extra_file_.close();
  }
};





// callback function for odom data
void RobotPrecisionEKFNode::odomCallback(const OdomConstPtr& odom)
{
  odom_callback_counter_++;
  odom_stamp_ = odom->header.stamp;
  odom_time_  = Time::now();
  
  double v, w, vR, vL;
  v = odom->twist.twist.linear.x;
  w = odom->twist.twist.angular.z;
  vR = v + ODOM_TRACK/2*w;
  vL = v - ODOM_TRACK/2*w;
  
  ekf_filter_->measurementUpdateOdom(vR, vL);
  
  cout << endl << endl;
  cout << "Encoders:" << endl;
  cout << "v: " << v << " w: " << w << endl
       << "vR: " << vR << " vL: " << vL << endl;
  cout << "Encoder Update: " << endl;
  cout << " Posterior Mean = " << endl << ekf_filter_->getMean() << endl
       << " Covariance = " << endl << ekf_filter_->getCovariance() << "" << endl;
};


// callback function for imu data
void RobotPrecisionEKFNode::imuCallback(const ImuConstPtr& imu)
{
  imu_callback_counter_++;
};

// callback function for GPS data
void RobotPrecisionEKFNode::gpsCallback(const GpsConstPtr& gps)
{
  gps_callback_counter_++;
  gps_stamp_ = gps->header.stamp;
  gps_time_  = Time::now();

  ekf_filter_->measurementUpdateGPS(gps->pose.position.x,gps->pose.position.y);
  
  cout << endl << endl;
  cout << "GPS Update: " << endl;
  cout << " Posterior Mean = " << endl << ekf_filter_->getMean() << endl
       << " Covariance = " << endl << ekf_filter_->getCovariance() << "" << endl;
  
  // Once the GPS message arrives, publish the updated state!
  publish();
};


// filter loop
void RobotPrecisionEKFNode::spin(const ros::TimerEvent& e)
{
  ROS_INFO("Spin function at time %f", ros::Time::now().toSec());
  
  ekf_filter_->systemUpdate();
  // TODO: Update the measurements here?? Or in the callbacks?? I just dont know!
  // And where should I publish the message?? 
  // Idea: Maybe I'll do the system update periodically in HERE...
  //     but then I'll only publish after the GPS measurment update is received
  
  cout << endl << endl;
  cout << "System Update: " << endl;
  cout << " Posterior Mean = " << endl << ekf_filter_->getMean() << endl
       << " Covariance = " << endl << ekf_filter_->getCovariance() << "" << endl;
       
  // The system will spin whether or not any measurements are received
};


void RobotPrecisionEKFNode::publish()
{
  MatrixWrapper::ColumnVector mean = ekf_filter_->getMean();
  MatrixWrapper::SymmetricMatrix cov = ekf_filter_->getCovariance();
  
  geometry_msgs::PoseWithCovarianceStamped p;
  // Fill in the header
  p.header.frame_id = global_frame_id_;
  p.header.stamp = gps_stamp_;
  // Copy in the pose
  p.pose.pose.position.x = mean(1);
  p.pose.pose.position.y = mean(2);
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(mean(3)), p.pose.pose.orientation);
  // Copy in the covariance
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      p.pose.covariance[6*i+j] = cov(i+1,j+1);
    }
  }
  p.pose.covariance[6*5+5] = cov(3,3);
  pose_pub_.publish(p);
  
  // subtracting base to odom from map to base and send map to odom instead
  tf::Stamped<tf::Pose> odom_to_map;
  try
  {
    tf::Transform tmp_tf(tf::createQuaternionFromYaw(mean(3)),
                         tf::Vector3(mean(1),
                                     mean(2),
                                     0.0));
    tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                          gps_stamp_,
                                          base_frame_id_);
    // TODO: (ejk) Use a message filter instead of waiting for the odom->base_link transform
    // Also, note that the rosrun tf view_frames program wasnt very happy with this tf... :(
    this->tf_->waitForTransform(odom_frame_id_, base_frame_id_, gps_stamp_, ros::Duration(0.1));
    this->tf_->transformPose(odom_frame_id_,
                             tmp_tf_stamped,
                             odom_to_map);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to subtract base to odom transform (%s)", e.what());
    return;
  }

  latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                             tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used
  ros::Time transform_expiration = (gps_stamp_ +
                                    transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                      transform_expiration,
                                      global_frame_id_, odom_frame_id_);
  this->tfb_->sendTransform(tmp_tf_stamped);
}


// ----------
// -- MAIN --
// ----------
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_precision_ekf");

  // create filter class
  RobotPrecisionEKFNode ekf_filter_node;

  ros::spin();
  
  return 0;
}
