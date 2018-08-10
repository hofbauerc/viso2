/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#include <libviso2/viso_stereo.h>

using namespace std;

VisualOdometryStereo::VisualOdometryStereo (parameters param) : param(param), VisualOdometry(param) {
  matcher->setIntrinsics(param.calib.f,param.calib.cu,param.calib.cv,param.base);
}

VisualOdometryStereo::~VisualOdometryStereo() {
}

bool VisualOdometryStereo::process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace) {
  matcher->pushBack(I1,I2,dims,replace);
  if (Tr_valid) matcher->matchFeatures(2,&Tr_delta);
  else          matcher->matchFeatures(2);
  matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
  p_matched = matcher->getMatches();
  return updateMotion();
}

vector<double> VisualOdometryStereo::estimateMotion (vector<Matcher::p_match> p_matched) {
  
  // return value
  bool success = true;
  
  // compute minimum distance for RANSAC samples
  double width=0,height=0;
  for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
    if (it->u1c>width)  width  = it->u1c;
    if (it->v1c>height) height = it->v1c;
  }
  double min_dist = min(width,height)/3.0;
  
  // get number of matches
  int32_t N  = p_matched.size();
  if (N<6)
    return vector<double>();

  // allocate dynamic memory
  X          = new double[N];
  Y          = new double[N];
  Z          = new double[N];
  J          = new double[4*N*6];
  p_predict  = new double[4*N];
  p_observe  = new double[4*N];
  p_residual = new double[4*N];

  // project matches of previous image into 3d
  for (int32_t i=0; i<N; i++) {
    double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
    X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
    Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
    Z[i] = param.calib.f*param.base/d;
  }

  // loop variables
  vector<double> tr_delta;
  vector<double> tr_delta_curr;
  tr_delta_curr.resize(6);
  
  // clear parameter vector
  inliers.clear();

  // initial RANSAC estimate
  for (int32_t k=0;k<param.ransac_iters;k++) {

    // draw random sample set
    vector<int32_t> active = getRandomSample(N,3);

    // clear parameter vector
    for (int32_t i=0; i<6; i++)
      tr_delta_curr[i] = 0;

    // minimize reprojection errors
    VisualOdometryStereo::result result = UPDATED;
    int32_t iter=0;
    while (result==UPDATED) {
      result = updateParameters(p_matched,active,tr_delta_curr,1,1e-6);
      if (iter++ > 20 || result==CONVERGED)
        break;
    }

    // overwrite best parameters if we have more inliers
    if (result!=FAILED) {
      vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
      if (inliers_curr.size()>inliers.size()) {
        inliers = inliers_curr;
        tr_delta = tr_delta_curr;
      }
    }
  }
  
  // final optimization (refinement)
  if (inliers.size()>=6) {
    int32_t iter=0;
    VisualOdometryStereo::result result = UPDATED;
    while (result==UPDATED) {     
      result = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
      if (iter++ > 100 || result==CONVERGED)
        break;
    }

    // not converged
    if (result!=CONVERGED)
      success = false;

  // not enough inliers
  } else {
    success = false;
  }


  // Try to estimate the covariance based on SVD
  {
      Matrix A(6, 6);
      // fill matrices A and B
      for (int32_t m=0; m<6; m++)
      {
          for (int32_t n=0; n<6; n++)
          {
              double a = 0;
              for (int32_t i=0; i<4*(int32_t)inliers.size(); i++)
              {
                  a += J[i*6+m]*J[i*6+n];
              }
              A.val[m][n] = a;
          }
      }

      // Compute the svd
      Matrix U(6,6);
      Matrix V(6,6);
      Matrix w(6,1);
      Matrix c0(6,1);
      Matrix c1(6,1);
      A.svd(U, c0, V);

      // Covariances based on singular values
      for (int32_t m=0; m<6; m++)
          c0.val[m][0] = param.cov_svd_factor / sqrt(c0.val[m][0]);

      // Absolut values of V
      for (int32_t m=0; m<6; m++)
          for (int32_t n=0; n<6; n++)
              V.val[m][n] = fabs(V.val[m][n]);

      // Relate singular values with parameters
      c1 = V * c0;

      // Set the covariances
      if (covariance.size() != 6)
          covariance.resize(6, 0.0);

      for (int32_t m=0; m<6; m++)
          covariance[m] = c1.val[m][0];
  }

//  // Try to estimate the covariance
//  {
//      if (success)
//      {
//          u_int32_t nofTrials = 400;
//
//          // Initialize the result
//          vector<double> tr_delta_samp;
//          vector<double> tr_delta_sum1;
//          vector<double> tr_delta_sum2;
//          vector<double> tr_delta_mean;
//          vector<double> tr_delta_vari;
//          tr_delta_samp.resize(6);
//          tr_delta_sum1.resize(6, 0.0);
//          tr_delta_sum2.resize(6, 0.0);
//          tr_delta_mean.resize(6, 0.0);
//          tr_delta_vari.resize(6, 0.0);
//
//          // Initialize the noisy matches
//          vector<Matcher::p_match> p_matched_noisy;
//          p_matched_noisy.resize(p_matched.size());
//
//          for (size_t i = 0; i < p_matched.size(); ++i)
//              p_matched_noisy[i] = p_matched[i];
//
//          // Open output file
//
//          // Open the file for binary writing
//          std::ofstream paramsstream;
//          std::stringstream fn;
//          fn.str("");
//          fn << "/home/administrator/data/DEBUG/VO_covariance_tests/data/params_" << rand() << ".txt";
//          paramsstream.open(fn.str().c_str(), std::ios::out | std::ios::trunc);
//          if (!paramsstream.is_open())
//              std::cout << "Opening estimates file failed!" << std::endl;
//
//          // Perform the trials
//          for (u_int32_t ell = 0; ell < nofTrials; ++ell)
//          {
//              // Reset
//              for (int32_t i=0; i<6; i++)
//                  tr_delta_samp[i] = tr_delta[i];
//
//              // Create a random sample by adding noise to inliers
//              for (size_t i = 0; i < inliers.size(); ++i)
//              {
//                  int32_t idx = inliers[i];
//
//                  p_matched_noisy[idx].u1c = p_matched[idx].u1c + (float)(((rand() % 401) - 200)) * 0.01;
//                  p_matched_noisy[idx].u1p = p_matched[idx].u1p + (float)(((rand() % 401) - 200)) * 0.01;
//                  p_matched_noisy[idx].u2c = p_matched[idx].u2c + (float)(((rand() % 401) - 200)) * 0.01;
//                  p_matched_noisy[idx].u2p = p_matched[idx].u2p + (float)(((rand() % 401) - 200)) * 0.01;
//
//                  p_matched_noisy[idx].v1c = p_matched[idx].v1c + (float)(((rand() % 401) - 200)) * 0.01;
//                  p_matched_noisy[idx].v1p = p_matched[idx].v1p + (float)(((rand() % 401) - 200)) * 0.01;
//                  p_matched_noisy[idx].v2c = p_matched[idx].v2c + (float)(((rand() % 401) - 200)) * 0.01;
//                  p_matched_noisy[idx].v2p = p_matched[idx].v2p + (float)(((rand() % 401) - 200)) * 0.01;
//
//                  double d = max(p_matched_noisy[idx].u1p - p_matched_noisy[idx].u2p,1.0f);
//                  X[idx] = (p_matched_noisy[idx].u1p-param.calib.cu)*param.base/d;
//                  Y[idx] = (p_matched_noisy[idx].v1p-param.calib.cv)*param.base/d;
//                  Z[idx] = param.calib.f*param.base/d;
//              }
//
//              // Optimization
//              int32_t iter=0;
//              VisualOdometryStereo::result result = UPDATED;
//              while (result==UPDATED)
//              {
//                  result = updateParameters(p_matched_noisy,inliers,tr_delta_samp,1,1e-8);
//                  if (iter++ > 50 || result==CONVERGED)
//                      break;
//              }
//
//              // Store the result
//              for (int32_t i=0; i<6; i++)
//              {
//                  tr_delta_sum1[i] += tr_delta_samp[i];
//                  tr_delta_sum2[i] += (tr_delta_samp[i] * tr_delta_samp[i]);
//
//                  paramsstream << std::fixed << std::setprecision(15) << tr_delta_samp[i] << " ";
//              }
//              paramsstream << iter << " " << inliers.size();
//              paramsstream << std::endl;
//          }
//
//          // Mean and variance
//          double scale = 1.0 / ((double)nofTrials);
//
//          for (int32_t i=0; i<6; i++)
//          {
//              tr_delta_mean[i] = tr_delta_sum1[i] * scale;
//              tr_delta_vari[i] = tr_delta_sum2[i] * scale - tr_delta_mean[i] * tr_delta_mean[i];
//
//              paramsstream << std::fixed << std::setprecision(15) << tr_delta_mean[i] << " ";
//              paramsstream << std::fixed << std::setprecision(15) << tr_delta_vari[i] << " ";
//          }
//          paramsstream << std::endl;
//
//          // Check if the writing was successful
//          if (!paramsstream.good())
//              std::cout << "Writing file failed!" << std::endl;
//
//          // Close the file
//          paramsstream.close();
//
//          // Write file for checking correlation of nof inliers and variances
//          fn.str("");
//          fn << "/home/administrator/data/DEBUG/VO_covariance_tests/correlation.txt";
//          std::ofstream corrstream;
//          corrstream.open(fn.str().c_str(), std::ios::out | std::ios::app);
//
//          if (!corrstream.is_open())
//              std::cout << "Opening correlation file failed!" << std::endl;
//
//          // Write inliers and variances
//          corrstream << inliers.size() << " ";
//          for (int32_t i=0; i<6; i++)
//              corrstream << std::fixed << std::setprecision(15) << tr_delta_vari[i] << " ";
//
//          corrstream << std::endl;
//
//          // Check if the writing was successful
//          if (!corrstream.good())
//              std::cout << "Writing correlation file failed!" << std::endl;
//
//          // Close the file
//          corrstream.close();
//      }
//  }

//  {
//      // Disturb the result and check the resulting residuals
//      double dMin[] = {-0.05,   -0.05,   -0.05,   -0.1,   -0.1,   -0.1};
//      double dMax[] = { 0.05,    0.05,    0.05,    0.1,    0.1,    0.1};
//      double ds[]   = { 0.0025,  0.0025,  0.0025,  0.005,  0.005,  0.005};
//
//      vector<double> tr_delta_samp;
//      tr_delta_samp.resize(6);
//
//      vector<double> residuals;
//      residuals.resize(4);
//
//      // Open the file for binary writing
//      std::ofstream deltastream;
//      std::stringstream fn;
//      fn.str("");
//      fn << "/home/administrator/data/DEBUG/VO_covariance_tests/deltas/deltas_" << rand() << ".txt";
//      deltastream.open(fn.str().c_str(), std::ios::out | std::ios::trunc);
//      if (!deltastream.is_open())
//          std::cout << "Opening delta file failed!" << std::endl;
//
//      for (int32_t i = 0; i < 6; ++i)
//      {
//          double delta = dMin[i];
//
//          deltastream << i << std::endl;
//
//          // for all offsets in the range
//          while (delta < dMax[i])
//          {
//              // Create the transform
//              for (int32_t j = 0; j < 6; ++j)
//                  tr_delta_samp[j] = tr_delta[j];
//
//              tr_delta_samp[i] += delta;
//
//              // Update the transform (which computes the residuals
//              updateParameters(p_matched,inliers,tr_delta_samp,1,1e-8);
//
//              // Compute the mean residuals
//              // for all observations do
//              for (int32_t k=0; k<4; k++)
//                  residuals[k] = 0.0;
//
//              for (int32_t k=0; k<(int32_t)inliers.size(); k++)
//              {
//                  // set residuals
//                  residuals[0] += fabs(p_residual[4*k+0]);
//                  residuals[1] += fabs(p_residual[4*k+1]);
//                  residuals[2] += fabs(p_residual[4*k+2]);
//                  residuals[3] += fabs(p_residual[4*k+3]);
//              }
//
//              double scale = 1.0 / ((double)inliers.size());
//              for (int32_t k=0; k<4; k++)
//                  residuals[k] *= scale;
//
//              // Write to the file
//              deltastream << delta;
//              for (int32_t k=0; k<4; k++)
//                  deltastream << " " << residuals[k];
//              deltastream << std::endl;
//
//              // Update the parameter offset
//              delta += ds[i];
//          }
//
//          deltastream << std::endl;
//      }
//
//
//      // Check if the writing was successful
//      if (!deltastream.good())
//          std::cout << "Writing delta file failed!" << std::endl;
//
//      // Close the file
//      deltastream.close();
//  }

  // release dynamic memory
  delete[] X;
  delete[] Y;
  delete[] Z;
  delete[] J;
  delete[] p_predict;
  delete[] p_observe;
  delete[] p_residual;
  
  // parameter estimate succeeded?
  if (success) return tr_delta;
  else         return vector<double>();
}

vector<int32_t> VisualOdometryStereo::getInlier(vector<Matcher::p_match> &p_matched,vector<double> &tr) {

  // mark all observations active
  vector<int32_t> active;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    active.push_back(i);

  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian(tr,active);

  // compute inliers
  vector<int32_t> inliers;
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
    if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
        pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
      inliers.push_back(i);
  return inliers;
}

VisualOdometryStereo::result VisualOdometryStereo::updateParameters(vector<Matcher::p_match> &p_matched,vector<int32_t> &active,vector<double> &tr,double step_size,double eps) {
  
  // we need at least 3 observations
  if (active.size()<3)
    return FAILED;
  
  // extract observations and compute predictions
  computeObservations(p_matched,active);
  computeResidualsAndJacobian(tr,active);

  // init
  Matrix A(6,6);
  Matrix B(6,1);

  // fill matrices A and B
  for (int32_t m=0; m<6; m++) {
    for (int32_t n=0; n<6; n++) {
      double a = 0;
      for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
        a += J[i*6+m]*J[i*6+n];
      }
      A.val[m][n] = a;
    }
    double b = 0;
    for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
      b += J[i*6+m]*(p_residual[i]);
    }
    B.val[m][0] = b;
  }

  // perform elimination
  if (B.solve(A)) {
    bool converged = true;
    for (int32_t m=0; m<6; m++) {
      tr[m] += step_size*B.val[m][0];
      if (fabs(B.val[m][0])>eps)
        converged = false;
    }
    if (converged)
      return CONVERGED;
    else
      return UPDATED;
  } else {
    return FAILED;
  }
}

void VisualOdometryStereo::computeObservations(vector<Matcher::p_match> &p_matched,vector<int32_t> &active) {

  // set all observations
  for (int32_t i=0; i<(int32_t)active.size(); i++) {
    p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
    p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
    p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
    p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
  }
}

void VisualOdometryStereo::computeResidualsAndJacobian(vector<double> &tr,vector<int32_t> &active) {

  // extract motion parameters
  double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
  double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

  // precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

  // compute rotation matrix and derivatives
  double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
  double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
  double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
  double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
  double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
  double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
  double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
  double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
  double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;

  // for all observations do
  for (int32_t i=0; i<(int32_t)active.size(); i++) {

    // get 3d point in previous coordinate system
    X1p = X[active[i]];
    Y1p = Y[active[i]];
    Z1p = Z[active[i]];

    // compute 3d point in current left coordinate system
    X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
    Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
    Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;
    
    // weighting
    double weight = 1.0;
    if (param.reweighting)
      weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);
    
    // compute 3d point in current right coordinate system
    X2c = X1c-param.base;

    // for all paramters do
    for (int32_t j=0; j<6; j++) {

      // derivatives of 3d pt. in curr. left coordinates wrt. param j
      switch (j) {
        case 0: X1cd = 0;
                Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                break;
        case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                break;
        case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                Y1cd = rdrz10*X1p+rdrz11*Y1p;
                Z1cd = rdrz20*X1p+rdrz21*Y1p;
                break;
        case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
        case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
        case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
      }

      // set jacobian entries (project via K)
      J[(4*i+0)*6+j] = weight*param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
      J[(4*i+1)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
      J[(4*i+2)*6+j] = weight*param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
      J[(4*i+3)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
    }

    // set prediction (project via K)
    p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
    p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
    p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
    p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v
    
    // set residuals
    p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
    p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
    p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
    p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
  }
}

