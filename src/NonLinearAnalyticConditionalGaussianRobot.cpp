// Copyright (C) 2008 Wim Meeussen <meeussen at willowgarage com>
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

#include <robot_precision_ekf/NonLinearAnalyticConditionalGaussianRobot.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#define NUMCONDARGUMENTS_MOBILE 1
#define NUMSTATES 5

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianRobot::NonLinearAnalyticConditionalGaussianRobot(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE),
      df(NUMSTATES,NUMSTATES)
  {
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
    state(1) += 0;
    state(2) += 0;
    state(3) += 0;
    state(4) += 0;
    state(5) += 0;
    return state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianRobot::dfGet(unsigned int i) const
  {
    if (i==0)//derivative to the first conditional argument (x)
    {
      double vel_trans = ConditionalArgumentGet(1)(1);
      double yaw = ConditionalArgumentGet(0)(6);
      
      df(1,3)=-vel_trans*sin(yaw); 
      df(2,3)= vel_trans*cos(yaw);

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

