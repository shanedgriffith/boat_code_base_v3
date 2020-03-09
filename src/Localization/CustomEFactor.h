#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/EssentialMatrix.h>

#include <gtsam/base/numericalDerivative.h>

#include <cmath>


class CustomEFactor: public gtsam::NoiseModelFactor1<gtsam::EssentialMatrix>
{
protected:
    
    gtsam::Vector3 p0h_, p1h_;
    
public:
    
    typedef gtsam::NoiseModelFactor1<gtsam::EssentialMatrix> Base;
    
    typedef CustomEFactor This;
    
    typedef boost::shared_ptr<This> shared_ptr;
    
    CustomEFactor(){}
    
    CustomEFactor(gtsam::Key eKey,
                  const gtsam::Point2& p0,
                  const gtsam::Point2& p1,
                  const gtsam::SharedNoiseModel& model) :
    p0h_(gtsam::EssentialMatrix::Homogeneous(p0)),
    p1h_(gtsam::EssentialMatrix::Homogeneous(p1)),
    Base(model, eKey) {}
    
    virtual ~CustomEFactor() {}
    
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                                  gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const
    {
        std::cout << s << "CustomEFactor, points: " << p0h_ << ", " << p1h_ << " ";
        Base::print("", keyFormatter);
    }
    
    virtual bool equals(const gtsam::NonlinearFactor& p, double tol = 1e-9) const
    {
        return false;
//        const This *e = dynamic_cast<const This*>(&p);
//        return e
//        && Base::equals(p, tol)
//        && std::fabs(p0h_.x()-e->p0h_.x()) < 0.01
//        && std::fabs(p0h_.y()-e->p0h_.y()) < 0.01
//        && std::fabs(p1h_.x()-e->p1h_.x()) < 0.01
//        && std::fabs(p1h_.y()-e->p1h_.y()) < 0.01 ;
    }
    
    static
    gtsam::Vector
    getError(const gtsam::EssentialMatrix& E, const gtsam::Vector3 p0h, const gtsam::Vector3 p1h)
    {
        gtsam::Vector3 v01 = E.matrix() * p0h;
        double dist01 = v01.dot(p1h) / sqrt(v01.x() * v01.x() + v01.y() * v01.y());
        return (gtsam::Vector(1) << dist01).finished();
    }
    
    gtsam::Vector
    evaluateError(const gtsam::EssentialMatrix& E,
                  boost::optional<gtsam::Matrix&> H = boost::none) const
    {
        if(H)
        {
            *H = gtsam::numericalDerivative11<gtsam::Vector, gtsam::EssentialMatrix>(boost::bind(&getError, _1, p0h_, p1h_), E, 0.0001);
        }
        
        return getError(E, p0h_, p1h_);
    }
};
