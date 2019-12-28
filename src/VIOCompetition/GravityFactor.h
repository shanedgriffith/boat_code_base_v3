#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/base/numericalDerivative.h>


/*
 gravity = acc_reading_t - (bias + acceleration_t)
 orientation_t * gravity = (0, 9.80655, 0)
 */
class GravityFactor: public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::Rot3> {

private:
    
    typedef GravityFactor This;
    typedef gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::Rot3> Base;
    
    const gtsam::Vector3 acc_reading_;

public:
    
    typedef typename boost::shared_ptr<GravityFactor> shared_ptr;
    
    GravityFactor() {}
    
    GravityFactor(gtsam::Key acc_t, gtsam::Key acc_bias, gtsam::Key orient_t, gtsam::Vector3 acc_reading, const gtsam::SharedNoiseModel& model):
    Base(model, acc_t, acc_bias, orient_t), acc_reading_(acc_reading) {}
    
    virtual ~GravityFactor() {}
    
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                                  gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "GravityFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ","
        << keyFormatter(this->key3()) << ")\n";
        this->noiseModel_->print("  noise model: ");
    }
    
    static gtsam::Vector LinearAcceleration(const gtsam::Vector3& acc_t, const gtsam::Vector3& bias_acc, const gtsam::Rot3& orient_t, const gtsam::Vector3& acc_reading)
    {
        const gtsam::Vector3 gravity(0, 9.80655, 0);
        gtsam::Vector3 remainder = acc_reading - (acc_t + bias_acc);
        gtsam::Vector3 expected = orient_t.matrix() * gravity;
        return expected - remainder;
    }
    
    gtsam::Vector evaluateError(const gtsam::Vector3& acc_t, const gtsam::Vector3& bias_acc, const gtsam::Rot3& orient_t,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none,
                                boost::optional<gtsam::Matrix&> H3 = boost::none) const
    {
        if(H1)
        {
            gtsam::Matrix D1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(boost::bind(GravityFactor::LinearAcceleration, _1, bias_acc, orient_t, acc_reading_), acc_t);
            gtsam::Matrix D2 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Vector3>(boost::bind(GravityFactor::LinearAcceleration, acc_t, _1, orient_t, acc_reading_), bias_acc);
            gtsam::Matrix D3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Rot3>(boost::bind(GravityFactor::LinearAcceleration, acc_t, bias_acc, _1, acc_reading_), orient_t);
            
            H1 = D1;
            H2 = D2;
            H3 = D3;
        }
        
        return LinearAcceleration(acc_t, bias_acc, orient_t, acc_reading_);
    }
};



























