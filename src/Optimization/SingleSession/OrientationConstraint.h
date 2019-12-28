#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


class OrientationConstraint: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
    
public:
    
    typedef gtsam::Pose3 T;
    
private:
    
    typedef OrientationConstraint This;
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;
    
    gtsam::Rot3 measured_; //measured yaw, roll and pitch unchanged.

public:
    typedef typename boost::shared_ptr<OrientationConstraint> shared_ptr;
    
    OrientationConstraint() {}
    
    //the key for a between factor is just the label that identifies it
    //specify 1, 2, and 3 in reverse chronological order.
    //i.e., x_t+1, x_t, and then v_t+1
    OrientationConstraint(gtsam::Key j1, gtsam::Key j2, const gtsam::Rot3& measured, const gtsam::SharedNoiseModel& model):
    Base(model, j1, j2), measured_(measured) {}
    
    virtual ~OrientationConstraint() {}
    
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                                  gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "BetweenFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ")\n";
        measured_.print("  measured: ");
        this->noiseModel_->print("  noise model: ");
    }
    
    gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none) const
    {
        if(H1)
        {
            //TODO.
            gtsam::Matrix m1, m2, r, dr, top;
            gtsam::Pose3 diff = p1.between(p2, m1, m2);
            const gtsam::Rot3& rot = diff.rotation(r);
            gtsam::Rot3 diffr = rot.between(measured_, dr);
            gtsam::Vector3 error = gtsam::Rot3::Logmap(diffr, top);
            gtsam::Matrix combined = top * dr * r;
            *H1 = combined * m1;
            *H2 = combined * m2;
            return error;
        }
        else
        {
            gtsam::Pose3 diff = p1.between(p2);
            gtsam::Rot3 diffr = diff.rotation().between(measured_);
            return gtsam::Rot3::Logmap(diffr);
        }
    }
};

