// Copyright (C) 2006 Klaas Gadeyne <first dot last at gmail dot com>
//                    Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#ifndef __EKF_CONST_
#define __EKF_CONST_

#include <cmath>

// To use measurements specify 1
// To do deadreckoning (no measurements) specify 0
#define USE_MEASUREMENTS 0

#define DELTA_T 0.05	        // Delta t (for discretisation)

// Sizes
#define STATE_SIZE 5 //state: x,y,theta,vel,omg
#define INPUT_SIZE 0 //input: no inputs

// Prior:
// Initial estimate of position and orientation
#define PRIOR_MU_X 0
#define PRIOR_MU_Y 0
#define PRIOR_MU_THETA 0	//M_PI/4
#define PRIOR_MU_VEL 0
#define PRIOR_MU_OMG 0
// Initial covariances of position and orientation
#define PRIOR_COV_X pow(100,2)
#define PRIOR_COV_Y pow(100,2)
#define PRIOR_COV_THETA pow(M_PI,2)
#define PRIOR_COV_VEL pow(10,2)
#define PRIOR_COV_OMG pow(10,2)

// System Noise
#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_THETA 0.0
#define MU_SYSTEM_NOISE_VEL 0.0
#define MU_SYSTEM_NOISE_OMG 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(2*M_PI/180,2)
#define SIGMA_SYSTEM_NOISE_VEL pow(0.05,2)
#define SIGMA_SYSTEM_NOISE_OMG pow(0.05,2)

// GPS Measurement
#define GPS_LEVERARM_X -0.45 // TODO: Pull the leverarm from the base_link->base_gps transform
#define GPS_LEVERARM_Y 0.0
#define GPS_MEAS_SIZE 2  //Measurement: [x; y]
#define GPS_SIGMA_MEAS_NOISE_X pow(0.05,2)
#define GPS_SIGMA_MEAS_NOISE_Y pow(0.05,2)
#define GPS_MU_MEAS_NOISE_X 0.0
#define GPS_MU_MEAS_NOISE_Y 0.0

// Odometry measurement
#define ODOM_TRACK 0.55 // TODO: Pull the track from the odometry parameter server
#define ODOM_MEAS_SIZE 2  //Measurement: [vR; vL]
#define ODOM_SIGMA_MEAS_NOISE_R pow(0.05,2)
#define ODOM_SIGMA_MEAS_NOISE_L pow(0.05,2)
#define ODOM_MU_MEAS_NOISE_X 0.0
#define ODOM_MU_MEAS_NOISE_Y 0.0

#endif //__EKF_CONST__
