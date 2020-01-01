#include "P3P.h"

/*
 Removed the TooN dependency.
 Added functions.
 */


/*
 * Copyright (c) 2011, Laurent Kneip, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * P3P.cpp
 *
 *  Created on: Nov 2, 2010
 *      Author: Laurent Kneip
 * Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences
 *   Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *              Absolute Camera Position and Orientation
 *
 *       Input: featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
 *              worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
 *              solutions: 3x16 matrix that will contain the solutions
 *                         form: [ 3x1 position(solution1) 3x3 orientation(solution1) 3x1 position(solution2) 3x3 orientation(solution2) ... ]
 *                         the obtained orientation matrices are defined as transforming points from the cam to the world frame
 *      Output: int: 0 if correct execution
 *                  -1 if world points aligned
 */

#include "P3P.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <complex>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

P3P::
P3P(const Camera& cam, const std::vector<gtsam::Point3>& p3d_subset, const std::vector<gtsam::Point2>& p2d_subset)
: cam_(cam)
, world_points_(p3d_subset)
, p2d_subset_(p2d_subset)
{
    
}

bool
P3P::
suitableSet()
{
    if(worldPoints.size() < 3)
    {
        return false;
    }
    
    Eigen::Vector3d P1 = world_points_[0].vector();
    Eigen::Vector3d P2 = world_points_[1].vector();
    Eigen::Vector3d P3 = world_points_[2].vector();
    
    Eigen::Vector3d temp1 = (P2 - P1);
    Eigen::Vector3d temp2 = (P3 - P1);
    
    if( (temp1.cross(temp2)).norm() == 0 )
    {
        return false;
    }
    return true;
}

gtsam::Pose3
P3P::
disambiguatePoses(const std::vector<gtsam::Pose3>& poses, gtsam::Point3& point3d, gtsam::Point2& point2d)
{
    int bestpidx = -1;
    double bestdist = std::numeric_limits<double>::max();
    for(int i=0; i<poses.size(); ++i)
    {
        gtsam::Point3 tfp = poses[i].transform_to(point3d);
        gtsam::Point2 res = cam_.ProjectToImage(tfp);
        double dist = res.distance(point2d);
        
        if(dist < bestdist)
        {
            bestdist = dist;
            bestpidx = i;
        }
    }
    
    return poses[bestpidx];
}

std::vector<gtsam::Vector3>
P3P::
pixelsToVectors()
{
    std::vector<gtsam::Vector3> feature_vectors(p2d_subset_.size());
    gtsam::PinholeCamera<gtsam::Cal3_S2> pc(gtsam::Pose3::identity(), *(cam_.GetGTSAMCam()));
    for(int j=0; j<p2d_subset_.size(); j++)
    {
        gtsam::Unit3 uv = pc.backprojectPointAtInfinity(p2d_subset_[j]);
        feature_vectors[j] = uv.unitVector();
    }
    return feature_vectors;
}

std::tuple<bool, gtsam::Pose3>
P3P::
run()
{
    if(not suitableSet())
    {
        return std::make_tuple(false, gtsam::Pose3::identity());
    }
    
    /*should have 4 points. the first three are used for the estimation. the fourth for disambiguation.*/
    std::vector<gtsam::Vector3> feature_vectors = pixelsToVectors();
    
    std::vector<gtsam::Pose3> poses = computePoses(feature_vectors); // TODO: break down this function.
    
    //NOTE: only the fourth point is used for disambiguation
    gtsam::Pose3 res = disambiguatePoses(poses, world_points_[3], p2d_subset_[3]);
    
    return std::make_tuple(true, res);
}

/*
 Computes up to 4 solutions. A fourth point needs to be projected for disambiguation.
 */
std::vector<gtsam::Pose3>
P3P::
computePoses( const std::vector<gtsam::Vector3>& featureVectors)
{
    // Extraction of world points
    Eigen::Vector3d P1 = world_points_[0].vector();
    Eigen::Vector3d P2 = world_points_[1].vector();
    Eigen::Vector3d P3 = world_points_[2].vector();
    
    // Extraction of feature vectors
    
    Eigen::Vector3d f1 = featureVectors[0].normalized();
    Eigen::Vector3d f2 = featureVectors[1].normalized();
    Eigen::Vector3d f3 = featureVectors[2].normalized();
    
    // Creation of intermediate camera frame
    
    Eigen::Vector3d e1 = f1;
    Eigen::Vector3d e3 = f1.cross(f2);
    e3.normalize();
    Eigen::Vector3d e2 = (e3.cross(e1)).normalized(); //normalization may be unnecessary
    
    Eigen::Matrix3d T;
    T << e1, e2, e3;
    
    f3 = T.transpose()*f3;
    
    // Reinforce that f3[2] > 0 for having theta in [0;pi]
    
    if( f3[2] > 0 )
    {
        f1 = featureVectors[1];
        f2 = featureVectors[0];
        f3 = featureVectors[2];
        
        e1 = f1;
        e3 = (f1.cross(f2)).normalized();
        e2 = e3.cross(e1);
        
        T << e1, e2, e3;
        
        f3 = T.transpose()*f3;
        
        P1 = world_points_[1].vector();
        P2 = world_points_[0].vector();
        P3 = world_points_[2].vector();
    }
    
    // Creation of intermediate world frame
    
    Eigen::Vector3d n1 = (P2-P1).normalized();
    Eigen::Vector3d n3 = (n1.cross(P3-P1)).normalized();
    Eigen::Vector3d n2 = n3.cross(n1);
    
    Eigen::Matrix3d N;
    N << n1, n2, n3;
    
    // Extraction of known parameters
    
    P3 = N.transpose()*(P3-P1);
    
    double d_12 = (P2-P1).norm();
    double f_1 = f3[0]/f3[2];
    double f_2 = f3[1]/f3[2];
    double p_1 = P3[0];
    double p_2 = P3[1];
    
    double cos_beta = f1.dot(f2);
    double b = 1/(1-pow(cos_beta,2)) - 1;
    
    if (cos_beta < 0)
        b = -sqrt(b);
    else
        b = sqrt(b);
    
    // Definition of temporary variables for avoiding multiple computation
    
    double f_1_pw2 = pow(f_1,2);
    double f_2_pw2 = pow(f_2,2);
    double p_1_pw2 = pow(p_1,2);
    double p_1_pw3 = p_1_pw2 * p_1;
    double p_1_pw4 = p_1_pw3 * p_1;
    double p_2_pw2 = pow(p_2,2);
    double p_2_pw3 = p_2_pw2 * p_2;
    double p_2_pw4 = p_2_pw3 * p_2;
    double d_12_pw2 = pow(d_12,2);
    double b_pw2 = pow(b,2);
    
    // Computation of factors of 4th degree polynomial
    
    Eigen::Matrix<double,5,1> factors;
    
    factors[0] = -f_2_pw2*p_2_pw4
    -p_2_pw4*f_1_pw2
    -p_2_pw4;
    
    factors[1] = 2*p_2_pw3*d_12*b
    +2*f_2_pw2*p_2_pw3*d_12*b
    -2*f_2*p_2_pw3*f_1*d_12;
    
    factors[2] = -f_2_pw2*p_2_pw2*p_1_pw2
    -f_2_pw2*p_2_pw2*d_12_pw2*b_pw2
    -f_2_pw2*p_2_pw2*d_12_pw2
    +f_2_pw2*p_2_pw4
    +p_2_pw4*f_1_pw2
    +2*p_1*p_2_pw2*d_12
    +2*f_1*f_2*p_1*p_2_pw2*d_12*b
    -p_2_pw2*p_1_pw2*f_1_pw2
    +2*p_1*p_2_pw2*f_2_pw2*d_12
    -p_2_pw2*d_12_pw2*b_pw2
    -2*p_1_pw2*p_2_pw2;
    
    factors[3] = 2*p_1_pw2*p_2*d_12*b
    +2*f_2*p_2_pw3*f_1*d_12
    -2*f_2_pw2*p_2_pw3*d_12*b
    -2*p_1*p_2*d_12_pw2*b;
    
    factors[4] = -2*f_2*p_2_pw2*f_1*p_1*d_12*b
    +f_2_pw2*p_2_pw2*d_12_pw2
    +2*p_1_pw3*d_12
    -p_1_pw2*d_12_pw2
    +f_2_pw2*p_2_pw2*p_1_pw2
    -p_1_pw4
    -2*f_2_pw2*p_2_pw2*p_1*d_12
    +p_2_pw2*f_1_pw2*p_1_pw2
    +f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;
    
    // Computation of roots
    
    Eigen::Vector4d realRoots;
    
    solveQuartic( factors, realRoots );
    
    // Backsubstitution of each solution
    std::vector<gtsam::Pose3> solutions;
    for(int i=0; i<4; ++i)
    {
        double cot_alpha = (-f_1*p_1/f_2-realRoots[i]*p_2+d_12*b)/(-f_1*realRoots[i]*p_2/f_2+p_1-d_12);
        
        double cos_theta = realRoots[i];
        double sin_theta = sqrt(1-pow(realRoots[i],2));
        double sin_alpha = sqrt(1/(pow(cot_alpha,2)+1));
        double cos_alpha = sqrt(1-pow(sin_alpha,2));
        
        if (cot_alpha < 0)
            cos_alpha = -cos_alpha;
        
        Eigen::Vector3d C;
        C << d_12*cos_alpha*(sin_alpha*b+cos_alpha),
             cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha),
             sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha);
        
        C = P1 + N*C;
        
        Eigen::Matrix3d R;
        R << -cos_alpha,		-sin_alpha*cos_theta,	-sin_alpha*sin_theta,
            sin_alpha,		-cos_alpha*cos_theta,	-cos_alpha*sin_theta,
            0,				-sin_theta,				cos_theta;
        
        R = N*R.transpose()*T.transpose();
        
        if(std::isnan(C.x()))
           continue;
           
        gtsam::Point3 t(C.x(), C.y(), C.z());
        gtsam::Rot3 rot(R);
        gtsam::Pose3 p(rot, t);
        solutions.push_back(p);
    }
    
    return solutions;
}

void P3P::solveQuartic( Eigen::Matrix<double,5,1> factors, Eigen::Vector4d & realRoots  )
{
    double A = factors[0];
    double B = factors[1];
    double C = factors[2];
    double D = factors[3];
    double E = factors[4];
    
    double A_pw2 = A*A;
    double B_pw2 = B*B;
    double A_pw3 = A_pw2*A;
    double B_pw3 = B_pw2*B;
    double A_pw4 = A_pw3*A;
    double B_pw4 = B_pw3*B;
    
    double alpha = -3*B_pw2/(8*A_pw2)+C/A;
    double beta = B_pw3/(8*A_pw3)-B*C/(2*A_pw2)+D/A;
    double gamma = -3*B_pw4/(256*A_pw4)+B_pw2*C/(16*A_pw3)-B*D/(4*A_pw2)+E/A;
    
    double alpha_pw2 = alpha*alpha;
    double alpha_pw3 = alpha_pw2*alpha;
    
    std::complex<double> P (-alpha_pw2/12-gamma,0);
    std::complex<double> Q (-alpha_pw3/108+alpha*gamma/3-pow(beta,2)/8,0);
    std::complex<double> R = -Q/2.0+sqrt(pow(Q,2.0)/4.0+pow(P,3.0)/27.0);
    
    std::complex<double> U = pow(R,(1.0/3.0));
    std::complex<double> y;
    
    if (U.real() == 0)
        y = -5.0*alpha/6.0-pow(Q,(1.0/3.0));
    else
        y = -5.0*alpha/6.0-P/(3.0*U)+U;
    
    std::complex<double> w = sqrt(alpha+2.0*y);
    
    std::complex<double> temp;
    
    temp = -B/(4.0*A) + 0.5*(w+sqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
    realRoots[0] = temp.real();
    temp = -B/(4.0*A) + 0.5*(w-sqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
    realRoots[1] = temp.real();
    temp = -B/(4.0*A) + 0.5*(-w+sqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
    realRoots[2] = temp.real();
    temp = -B/(4.0*A) + 0.5*(-w-sqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
    realRoots[3] = temp.real();
}
