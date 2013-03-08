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

#include <robot_precision_ekf/nonlinearanalyticconditionalgaussian_robot.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#include "angles/angles.h"
#define NUMCONDARGUMENTS 1
#define NUMSTATES 5

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianRobot::NonLinearAnalyticConditionalGaussianRobot(const Gaussian& additiveNoise, double timestep)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS),
      df(NUMSTATES,NUMSTATES)
  {
    dt = timestep;
    // initialize df matrix
    for (unsigned int i=1; i<=NUMSTATES; i++){
      for (unsigned int j=1; j<=NUMSTATES; j++){
        if (i==j) df(i,j) = 1;
        else df(i,j) = 0;
      }
    }
  }


  NonLinearAnalyticConditionalGaussianRobot::~NonLinearAnalyticConditionalGaussianRobot(){}

  ColumnVector NonLinearAnalyticConditionalGaussianRobot::ExpectedValueGet() const
  {
    ColumnVector state = ConditionalArgumentGet(0);
    // f(x) for a mobile robot, where x = [x;y;tht;vel;omg]
    double tht_mid = state(3)+state(5)*dt/2; // Theta mid-pt = Thtold + w*dt/2
    state(1) = state(1) + state(4)*dt*cos(tht_mid);
    state(2) = state(2) + state(4)*dt*sin(tht_mid);
    state(3) = angles::normalize_angle(state(3) + state(5)*dt);
    state(4) = state(4);
    state(5) = state(5);
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianRobot::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
    {
      // Jacobian F = df(x)/dx for a mobile robot, where x = [x;y;tht;vel;omg]
      ColumnVector state = ConditionalArgumentGet(0);
      
      double tht_mid = state(3)+state(5)*dt/2; // Theta mid-pt = Thtold + w*dt/2
      
      df(1,1) = 1.0;
      df(1,3) = -state(4)*dt*sin(tht_mid);
      df(1,4) = dt*cos(tht_mid);
      df(1,5) = -state(4)*dt*dt/2*sin(tht_mid);
      
      df(2,2) = 1.0;
      df(2,3) = state(4)*dt*cos(tht_mid);
      df(2,4) = dt*sin(tht_mid);
      df(2,5) = state(4)*dt*dt/2*cos(tht_mid);
      
      df(3,3) = 1.0;
      df(3,5) = dt;
      
      df(4,4) = 1.0;
      
      df(5,5) = 1.0;

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

