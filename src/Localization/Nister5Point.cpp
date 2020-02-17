#include "Nister5Point.h"

#include "5point/Polynomial.h"
#include "5point/Rpoly.h"

#include <gtsam/geometry/triangulation.h>
#include <gtsam/geometry/Cal3_S2.h>

#include <stdio.h>

using namespace Eigen;

Nister5Point::
Nister5Point(const Camera& cam, const std::vector<gtsam::Point2>& p2d0_subset, const std::vector<gtsam::Point2>& p2d1_subset)
: cam_(cam)
, p2d0_subset_(p2d0_subset)
, p2d1_subset_(p2d1_subset)
{}

bool
Nister5Point::
suitableSet()
{
    if(p2d0_subset_.size() < 5)
    {
        return false;
    }
    return true;
}

std::vector<Nister5Point::EMatrix>
Nister5Point::
computePossibleSolutions()
{
    // F is a temp variable, not the F fundamental matrix
    Matrix<double, Dynamic, 9> F(p2d0_subset_.size(),9);
    for(int i=0; i < p2d0_subset_.size(); ++i)
    {
        const double& x1 = p2d0_subset_[i](0);
        const double& y1 = p2d0_subset_[i](1);
        
        const double& x2 = p2d1_subset_[i](0);
        const double& y2 = p2d1_subset_[i](1);
        F(i,0) = x1*x2;
        F(i,1) = x2*y1;
        F(i,2) = x2;
        F(i,3) = x1*y2;
        F(i,4) = y1*y2;
        F(i,5) = y2;
        F(i,6) = x1;
        F(i,7) = y1;
        F(i,8) = 1.0;
    }
    
    JacobiSVD<Matrix<double, Dynamic, 9> >    svd(F, ComputeFullV);
    
    const double e00 = svd.matrixV()(0,5),
    e01 = svd.matrixV()(1,5),
    e02 = svd.matrixV()(2,5),
    e03 = svd.matrixV()(3,5),
    e04 = svd.matrixV()(4,5),
    e05 = svd.matrixV()(5,5),
    e06 = svd.matrixV()(6,5),
    e07 = svd.matrixV()(7,5),
    e08 = svd.matrixV()(8,5),
    
    e10 = svd.matrixV()(0,6),
    e11 = svd.matrixV()(1,6),
    e12 = svd.matrixV()(2,6),
    e13 = svd.matrixV()(3,6),
    e14 = svd.matrixV()(4,6),
    e15 = svd.matrixV()(5,6),
    e16 = svd.matrixV()(6,6),
    e17 = svd.matrixV()(7,6),
    e18 = svd.matrixV()(8,6),
    
    e20 = svd.matrixV()(0,7),
    e21 = svd.matrixV()(1,7),
    e22 = svd.matrixV()(2,7),
    e23 = svd.matrixV()(3,7),
    e24 = svd.matrixV()(4,7),
    e25 = svd.matrixV()(5,7),
    e26 = svd.matrixV()(6,7),
    e27 = svd.matrixV()(7,7),
    e28 = svd.matrixV()(8,7),
    
    e30 = svd.matrixV()(0,8),
    e31 = svd.matrixV()(1,8),
    e32 = svd.matrixV()(2,8),
    e33 = svd.matrixV()(3,8),
    e34 = svd.matrixV()(4,8),
    e35 = svd.matrixV()(5,8),
    e36 = svd.matrixV()(6,8),
    e37 = svd.matrixV()(7,8),
    e38 = svd.matrixV()(8,8);
    
    // Out symbolic polynomial matrix
    PolyMatrix M(10,10);
    
    // This file is not pretty to look at ...
#include "5point/Mblock.h"
    
    // symbolic determinant using interpolation based on the papers:
    // "Symbolic Determinants: Calculating the Degree", http://www.cs.tamu.edu/academics/tr/tamu-cs-tr-2005-7-1
    // "Multivariate Determinants Through Univariate Interpolation", http://www.cs.tamu.edu/academics/tr/tamu-cs-tr-2005-7-2
    
    // max power of the determinant is x^10, so we need 11 points for interpolation
    // the 11 points are at x = [-5, -4 .... 4, 5], luckily there is no overflow at x^10
    
    Matrix<double, 11, 11>              X;
    Matrix<double, 11, 1>               b;
    Matrix<double, 10, 10, RowMajor>    ret_eval;
    X.col(0).fill(1);
    
    // first column of M is the lowest power
    for(int i=-5, j=0; i <= 5; ++i, ++j)
    {
        M.Eval(i, ret_eval.data());
        double t = i;
        for(int k=1; k < 11; ++k)
        {
            X(j,k) = t;
            t *= i;
        }
        b(j,0) = ret_eval.determinant();
    }
    
    // Using full pivot LU inverse, as partial pivot LU inverse (the default) generates less accurate inverses
    Matrix<double, 11, 1>   a = X.fullPivLu().inverse()*b;
    
    // Solve for z
    int degrees = 10;
    double coeffs[11];
    double zeror[11], zeroi[11];
    
    // rpoly_ak1 expects highest power first
    for(int i=0; i < a.size(); i++)
    {
        coeffs[i] = a(a.size()-i-1);
    }
    
    // Find roots of polynomial
    rpoly_ak1(coeffs, &degrees, zeror, zeroi);
    
    std::vector<EMatrix> sols;
    
    JacobiSVD<Matrix<double, 10, 10, RowMajor> >    svd2(10,10);
    for(int i=0; i < degrees; ++i)
    {
        if(zeroi[i] != 0)
        {
            continue;
        }
        
        double z = zeror[i];
        
        M.Eval(z, ret_eval.data());
        
        // Extract svd full V
        svd2.compute(ret_eval, ComputeFullV);
        
        // svd2.matrixV().col(9) represents
        // [x^3 , y^3 , x^2 y, xy^2 , x^2 , y^2 , xy, x, y, 1]^T
        
        // Scale it so the last element is 1, to get the correct answer
        double x = svd2.matrixV()(7,9) / svd2.matrixV()(9,9);
        double y = svd2.matrixV()(8,9) / svd2.matrixV()(9,9);
        
        EMatrix E;
        
        // Build the essential matrix from all the known x,y,z values
        E(0,0) = e00*x + e10*y + e20*z + e30;
        E(0,1) = e01*x + e11*y + e21*z + e31;
        E(0,2) = e02*x + e12*y + e22*z + e32;
        
        E(1,0) = e03*x + e13*y + e23*z + e33;
        E(1,1) = e04*x + e14*y + e24*z + e34;
        E(1,2) = e05*x + e15*y + e25*z + e35;
        
        E(2,0) = e06*x + e16*y + e26*z + e36;
        E(2,1) = e07*x + e17*y + e27*z + e37;
        E(2,2) = e08*x + e18*y + e28*z + e38;
        
        sols.push_back(E);
    }
    
    return sols;
}

// X is 4x1 is [x,y,z,w]
// P is 3x4 projection matrix
double
Nister5Point::
calcDepth(const Vector4d &X, const PMatrix &P)
{
    // back project
    Vector3d X2 = P*X;
    
    double det = P.block(0,0,3,3).determinant();
    double w = X2(2,0);
    double W = X(3,0);
    
    double a = P(0,2);
    double b = P(1,2);
    double c = P(2,2);
    
    double m3 = sqrt(a*a + b*b + c*c);  // 3rd column of M
    
    double sign;
    
    if(det > 0)
    {
        sign = 1;
    }
    else
    {
        sign = -1;
    }
    
    return (w/W)*(sign/m3);
}

Vector4d
Nister5Point::
triangulatePoint(const gtsam::Point2& a, const gtsam::Point2& b, const PMatrix &P1, const PMatrix &P2)
{
    Matrix4d A;
    
    double x1 = a.x();
    double y1 = a.y();
    double x2 = b.x();
    double y2 = b.y();
    
    A(0,0) = x1*P1(2,0) - P1(0,0);
    A(0,1) = x1*P1(2,1) - P1(0,1);
    A(0,2) = x1*P1(2,2) - P1(0,2);
    A(0,3) = x1*P1(2,3) - P1(0,3);
    
    A(1,0) = y1*P1(2,0) - P1(1,0);
    A(1,1) = y1*P1(2,1) - P1(1,1);
    A(1,2) = y1*P1(2,2) - P1(1,2);
    A(1,3) = y1*P1(2,3) - P1(1,3);
    
    A(2,0) = x2*P2(2,0) - P2(0,0);
    A(2,1) = x2*P2(2,1) - P2(0,1);
    A(2,2) = x2*P2(2,2) - P2(0,2);
    A(2,3) = x2*P2(2,3) - P2(0,3);
    
    A(3,0) = y2*P2(2,0) - P2(1,0);
    A(3,1) = y2*P2(2,1) - P2(1,1);
    A(3,2) = y2*P2(2,2) - P2(1,2);
    A(3,3) = y2*P2(2,3) - P2(1,3);
    
    JacobiSVD<Matrix4d> svd(A, ComputeFullV);
    
    return svd.matrixV().col(3);
}


std::vector<Nister5Point::PMatrix, Eigen::aligned_allocator<Nister5Point::PMatrix>>
Nister5Point::
projectionsFromEssential(const Nister5Point::EMatrix &E)
{   // four possible projection matrices from an essential matrix.
    std::vector<Nister5Point::PMatrix, Eigen::aligned_allocator<Nister5Point::PMatrix> > P(4);
    
    // Assumes input E is a rank 2 matrix, with equal singular values
    JacobiSVD<EMatrix> svd(E, ComputeFullU | ComputeFullV);
    const Matrix3d &U = svd.matrixU(),
    &V = svd.matrixV();
    Matrix3d W;
    
    // Find rotation, translation
    W.setZero();
    W(0,1) = -1.0;
    W(1,0) = 1.0;
    W(2,2) = 1.0;
    
    // Rotation
    Matrix3d R1 = U * W             * V.transpose();
    Matrix3d R2 = U * W.transpose() * V.transpose();
    
    P[0].block(0,0,3,3) = R1;
    P[1].block(0,0,3,3) = R1;
    P[2].block(0,0,3,3) = R2;
    P[3].block(0,0,3,3) = R2;
    
    // Translation
    P[0].col(3) =  U.col(2);
    P[1].col(3) = -U.col(2);
    P[2].col(3) =  U.col(2);
    P[3].col(3) = -U.col(2);
    
    return P;
}

std::tuple<bool, Nister5Point::PMatrix>
Nister5Point::
disambiguateSolutions(std::vector<Nister5Point::EMatrix> solutions)
{
    PMatrix P_ref = Nister5Point::PMatrix::Identity();
    
    for(int i=0; i<solutions.size(); ++i)
    {
        // Test to see if this E matrix is the correct one we're after
        std::vector<PMatrix, Eigen::aligned_allocator<Nister5Point::PMatrix>> P = projectionsFromEssential(solutions[i]);
        
        for(size_t j=0; j < P.size(); j++)
        {
            gtsam::EssentialMatrix pguess = convertTo(P[j]);
            
            gtsam::Pose3 p0;
            gtsam::Pose3 p1(pguess.rotation(), pguess.direction().point3());
            std::vector<gtsam::Pose3> poses = {p0, p1};
            
            int nin=0;
            for(int i=0; i<p2d0_subset_.size(); ++i)
            {
                gtsam::Point2Vector measurements = {
                    p2d0_subset_[i],
                    p2d1_subset_[i]
                };
                
                try{
                    gtsam::Point3 resp = gtsam::triangulatePoint3(poses, cam_.GetGTSAMCam(), measurements);
                    ++nin;
                }catch(std::exception e) {}
            }
            
            if(nin == 5)
            {
                return std::make_tuple(true, P[j]);
            }
        }
    }
    
    return std::make_tuple(false, P_ref);
}

gtsam::EssentialMatrix
Nister5Point::
convertTo(Nister5Point::PMatrix& P)
{
    Eigen::Matrix<double, 3, 3> R = P.block(0,0,3,3);
    Eigen::Matrix<double, 3, 1> t = P.block(0,3,3,1);
    gtsam::Unit3 tu(t);
    gtsam::Rot3 rot(R);
    return gtsam::EssentialMatrix(rot, tu);
}

std::tuple<bool, gtsam::EssentialMatrix>
Nister5Point::
run()
{
    if(not suitableSet())
    {
        return std::make_tuple(false, gtsam::EssentialMatrix());
    }
    
    std::vector<Nister5Point::EMatrix> solutions = computePossibleSolutions();
    
    if(solutions.size() == 0)
    {
        return std::make_tuple(false, gtsam::EssentialMatrix());
    }
    
    bool sol;
    PMatrix P;
    std::tie(sol, P) = disambiguateSolutions(solutions);
    
    if(sol)
    {
        gtsam::EssentialMatrix p = convertTo(P);
        
        gtsam::Vector3 r = gtsam::Rot3::Logmap(p.rotation());
        gtsam::Vector3 t = p.direction().point3();
        
        for(int i=0; i<3; ++i)
        {
            if(isnan(r(i)) or isnan(t(i)))
            {
                return std::make_tuple(false, gtsam::EssentialMatrix());
            }
        }
        
        return std::make_tuple(true, p);
    }
    
    return std::make_tuple(false, gtsam::EssentialMatrix());
}
