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

#include <robot_precision_ekf/nonlinearanalyticconditionalgaussian_3state.h>
#include <wrappers/rng/rng.h> // Wrapper around several rng
                              // libraries
#include "angles/angles.h"
#define NUMCONDARGUMENTS 2
#define NUMSTATES 3

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussian3State::NonLinearAnalyticConditionalGaussian3State(const Gaussian& additiveNoise, double timestep)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS),
      df(NUMSTATES,NUMSTATES),
      du(NUMSTATES,2)
  { 
    alpha_ = 0.0;
    epsilon_ = 1.0;
    
    dt = timestep;
    // initialize df matrix
    for (unsigned int i=1; i<=NUMSTATES; i++){
      for (unsigned int j=1; j<=NUMSTATES; j++){
        if (i==j) df(i,j) = 1;
        else df(i,j) = 0;
      }
    }
  }
  
  NonLinearAnalyticConditionalGaussian3State::~NonLinearAnalyticConditionalGaussian3State(){}

  void NonLinearAnalyticConditionalGaussian3State::setOdomNoise(double alpha, double epsilon)
  {
    alpha_ = alpha;
    epsilon_ = epsilon;
  }

  ColumnVector NonLinearAnalyticConditionalGaussian3State::ExpectedValueGet() const
  {
    double v,w,tht_mid;
    ColumnVector state, input;
    
    // f(x,u) for a mobile robot, where x = [x;y;tht]
    // and u = [vR;vL]
    
    state = ConditionalArgumentGet(0);
    input = ConditionalArgumentGet(1);
    v = (input(1)+input(2))/2;
    w = (input(1)-input(2))/ODOM_TRACK;
    tht_mid = state(3)+w*dt/2; // Theta mid-pt = Thtold + w*dt/2
    
    cout << endl << "3State: ExpectedValueGet!" << endl << state << input << endl;
    
    state(1) = state(1) + v*dt*cos(tht_mid);
    state(2) = state(2) + v*dt*sin(tht_mid);
    state(3) = angles::normalize_angle(state(3) + w*dt);
    
    cout << state << endl;
    
    return state + AdditiveNoiseMuGet();    
  }

  Matrix NonLinearAnalyticConditionalGaussian3State::dfGet(unsigned int i) const
  {
    double v,w,tht_mid;
    ColumnVector state, input;
    
    if (i==0)
    {  
        // Jacobian F = df(x,u)/dx for a mobile robot, where x = [x;y;tht] and u = [vR;vL]
        state = ConditionalArgumentGet(0);
        input = ConditionalArgumentGet(1);
        v = (input(1)+input(2))/2;
        w = (input(1)-input(2))/ODOM_TRACK;
        tht_mid = state(3)+w*dt/2; // Theta mid-pt = Thtold + w*dt/2
        
        df(1,1) = 1.0;
        df(1,3) = -v*dt*sin(tht_mid);
        
        df(2,2) = 1.0;
        df(2,3) = v*dt*cos(tht_mid);
        
        df(3,3) = 1.0;  
        
        cout << "3State: dfGet!" << endl << state << input << endl << df << endl;
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
  
  SymmetricMatrix NonLinearAnalyticConditionalGaussian3State::CovarianceGet() const
  {
    
    double v,w,tht_mid;
    ColumnVector state, input;
    
    // Jacobian V = df(x,u)/du for a mobile robot, where x = [x;y;tht] and u = [vR;vL]
    state = ConditionalArgumentGet(0);
    input = ConditionalArgumentGet(1);
    v = (input(1)+input(2))/2;
    w = (input(1)-input(2))/ODOM_TRACK;
    tht_mid = state(3)+w*dt/2; // Theta mid-pt = Thtold + w*dt/2
    
    du(1,1) = 0.5*cos(tht_mid)-v*dt/(2.0*ODOM_TRACK)*sin(tht_mid);
    du(1,2) = 0.5*cos(tht_mid)+v*dt/(2.0*ODOM_TRACK)*sin(tht_mid);
    
    du(2,1) = 0.5*sin(tht_mid)+v*dt/(2.0*ODOM_TRACK)*cos(tht_mid);
    du(2,2) = 0.5*sin(tht_mid)-v*dt/(2.0*ODOM_TRACK)*cos(tht_mid);
    
    du(3,1) =  dt/ODOM_TRACK;
    du(3,2) = -dt/ODOM_TRACK;
    
    cout << "3State: CovarianceGet!" << endl << state << input << endl << du << endl;
    
    // Perform the calculation to propagate noise
    SymmetricMatrix odom_noise(2);
    SymmetricMatrix Q_out;
    odom_noise = 0.0;
    odom_noise(1,1) = alpha_ * fabs(input(1)) + epsilon_;
    odom_noise(2,2) = alpha_ * fabs(input(2)) + epsilon_;
    Matrix sigma_temp = du * ( (Matrix)odom_noise * du.transpose() );
    cout << odom_noise << endl << sigma_temp << endl;
    sigma_temp += (Matrix)AdditiveNoiseSigmaGet();
    sigma_temp.convertToSymmetricMatrix(Q_out);
    cout << Q_out << endl;
    
    return Q_out;
  }

}//namespace BFL

