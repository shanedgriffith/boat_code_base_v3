#pragma once

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Point2.h>
#include "DataTypes/Camera.hpp"

#include <Eigen/StdVector>

//Nister's 5point algorithm to estimate the essential matrix. Implementation due to Nghia Ho.
class Nister5Point
{
private:
    
    typedef Eigen::Matrix<double, 3, 4>    PMatrix;
    typedef Eigen::Matrix3d                EMatrix;
    
    Eigen::Vector4d
    triangulatePoint(const gtsam::Point2& a, const gtsam::Point2& b, const PMatrix &P1, const PMatrix &P2);
    
    double
    calcDepth(const Eigen::Vector4d &X, const PMatrix &P);
    
    std::vector<PMatrix, Eigen::aligned_allocator<PMatrix> >
    projectionsFromEssential(const EMatrix &E);
    
    std::tuple<bool, PMatrix>
    disambiguateSolutions(std::vector<EMatrix> solutions);
    
    std::vector<EMatrix>
    computePossibleSolutions();
    
    gtsam::EssentialMatrix
    convertTo(PMatrix& P);
    
    bool
    suitableSet();
    
    const Camera& cam_;
    const std::vector<gtsam::Point2>& p2d0_subset_;
    const std::vector<gtsam::Point2>& p2d1_subset_;
    
public:
    
    
    Nister5Point(const Camera& cam, const std::vector<gtsam::Point2>& p2d0_subset, const std::vector<gtsam::Point2>& p2d1_subset);
    
    std::tuple<bool, gtsam::EssentialMatrix>
    run();
    
};
