// Copyright (C) 2013 EJ Kreinar, Case Western Reserve University
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

#include <robot_precision_ekf/nonlinearanalyticconditionalgaussian_gpsmeasurement.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#define NUMCONDARGUMENTS 1

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianGPSMeasurement::NonLinearAnalyticConditionalGaussianGPSMeasurement(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS),
      df(GPS_MEAS_SIZE,STATE_SIZE)
  {
    // initialize df matrix
    for (unsigned int i=1; i<=GPS_MEAS_SIZE; i++){
      for (unsigned int j=1; j<=STATE_SIZE; j++){
        if (i==j) df(i,j) = 1;
        else df(i,j) = 0;
      }
    }
  }


  NonLinearAnalyticConditionalGaussianGPSMeasurement::~NonLinearAnalyticConditionalGaussianGPSMeasurement(){}

  ColumnVector NonLinearAnalyticConditionalGaussianGPSMeasurement::ExpectedValueGet() const
  {
    // XY MEASUREMENT at Arbitrary relationship to origin (nonlinear)
    // y = [xgps    =   [x + xarm*cos(tht) - yarm*sin(tht) 
    //      ygps]        y + xarm*sin(tht) + yarm*cos(tht)]
    
    ColumnVector state = ConditionalArgumentGet(0);
    ColumnVector z(GPS_MEAS_SIZE);
    z(1) = state(1) + GPS_LEVERARM_X*cos(state(3)) - GPS_LEVERARM_Y*sin(state(3));
    z(2) = state(2) + GPS_LEVERARM_X*sin(state(3)) + GPS_LEVERARM_Y*cos(state(3));
    
    return z + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianGPSMeasurement::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
    {
      ColumnVector state = ConditionalArgumentGet(0);
      
      // XY Partial Derivative w.r.t. the State:
      df(1,1) = 1.0; df(1,3) = -GPS_LEVERARM_X*sin(state(3))-GPS_LEVERARM_Y*cos(state(3));
      df(2,2) = 1.0; df(2,3) =  GPS_LEVERARM_X*cos(state(3))-GPS_LEVERARM_Y*sin(state(3));
      // All other terms = 0.0
      
      return df;
    }
    else
    {
      if (i >= NumConditionalArgumentsGet())
      {
        cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
        exit(-BFL_ERRMISUSE);
      }
      else{
        cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
        exit(-BFL_ERRMISUSE);
      }
    }
  }

}//namespace BFL

