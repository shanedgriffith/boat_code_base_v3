#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/base/numericalDerivative.h>

#include <cmath>


class CustomEFactor2: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
protected:
    
    gtsam::Vector3 p0h_, p1h_;
    
public:
    
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;
    
    typedef CustomEFactor2 This;
    
    typedef boost::shared_ptr<This> shared_ptr;
    
    CustomEFactor2(){}
    
    CustomEFactor2(gtsam::Key poseKey1,
                   gtsam::Key poseKey2,
                  const gtsam::Point2& p0,
                  const gtsam::Point2& p1,
                  const gtsam::SharedNoiseModel& model) :
    p0h_(gtsam::EssentialMatrix::Homogeneous(p0)),
    p1h_(gtsam::EssentialMatrix::Homogeneous(p1)),
    Base(model, poseKey1, poseKey2) {}
    
    virtual ~CustomEFactor2() {}
    
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                                  gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const
    {
        std::cout << s << "CustomEFactor2, points: " << p0h_ << ", " << p1h_ << " ";
        Base::print("", keyFormatter);
    }
    
    virtual bool equals(const gtsam::NonlinearFactor& p, double tol = 1e-9) const
    {
        return false;
    }
    
    static
    gtsam::Vector
    getError(const gtsam::Pose3& p1, const gtsam::Pose3& p2, const gtsam::Vector3 p0h, const gtsam::Vector3 p1h)
    {
        gtsam::Pose3 btwn = p1.between(p2);
        gtsam::EssentialMatrix E = gtsam::EssentialMatrix::FromPose3(btwn);
        gtsam::Vector3 v01 = E.matrix() * p0h;
        double dist01 = v01.dot(p1h) / sqrt(v01.x() * v01.x() + v01.y() * v01.y());
        return (gtsam::Vector(1) << dist01).finished();
    }
    
    gtsam::Vector
    evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
                  boost::optional<gtsam::Matrix&> H1 = boost::none,
                  boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        if(H1)
        {
            *H1 = gtsam::numericalDerivative21<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(boost::bind(&getError, _1, _2, p0h_, p1h_), p1, p2, 0.0001);
            *H2 = gtsam::numericalDerivative22<gtsam::Vector, gtsam::Pose3, gtsam::Pose3>(boost::bind(&getError, _1, _2, p0h_, p1h_), p1, p2, 0.0001);
        }
        
        return getError(p1, p2, p0h_, p1h_);
    }
};
