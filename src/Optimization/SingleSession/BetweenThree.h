//
//  BetweenThree.h
//  BundleAdjustOneDataset
//
//  Created by Shane Griffith on 10/29/14.
//  Copyright (c) 2014 shane. All rights reserved.
//

#ifndef SRC_OPTIMIZATION_BETWEENTHREE_H_
#define SRC_OPTIMIZATION_BETWEENTHREE_H_

#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


/*
 Note: This factor is designed specifically for the boat due to the 
 assumption that only x,y, and yaw are the non-static variables.
 */
template<class Pose3>
class BetweenThree: public gtsam::NoiseModelFactor3<Pose3, Pose3, Pose3> {

public:
    
    typedef Pose3 T;
    
private:
    
    typedef BetweenThree This;
    typedef gtsam::NoiseModelFactor3<Pose3, Pose3, Pose3> Base;
    
    Pose3 measured_;
    double delta_t;
public:
    typedef typename boost::shared_ptr<BetweenThree> shared_ptr;
    
    BetweenThree() {}
    
    //the key for a between factor is just the label that identifies it
    //specify 1, 2, and 3 in reverse chronological order.
    //i.e., x_t+1, x_t, and then v_t+1
    BetweenThree(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, double delta_time, gtsam::Pose3 measured, const gtsam::SharedNoiseModel& model):
         Base(model, j1, j2, j3), measured_(measured), delta_t(delta_time){}
    
    virtual ~BetweenThree() {}
    
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "BetweenFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ","
        << keyFormatter(this->key3()) << ")\n";
        measured_.print("  measured: ");
        this->noiseModel_->print("  noise model: ");
    }
    
    
    gtsam::Vector evaluateError(const Pose3& p1, const Pose3& p2, const Pose3& p3,
                         boost::optional<gtsam::Matrix&> H1 = boost::none,
                         boost::optional<gtsam::Matrix&> H2 = boost::none,
                         boost::optional<gtsam::Matrix&> H3 = boost::none) const {
        Pose3 hx = p1.between(p2, H1, H2); // h(x)
        /*double weight = 1/delta_t;
        gtsam::Rot3 r=hx.rotation();
        //only the x,y, and yaw are modified. The other values shouldn't change (for the boat, anyway)
        hx = Pose3(gtsam::Rot3::Ypr(r.yaw()*weight,r.pitch(),r.roll()), gtsam::Point3(hx.x()*weight, hx.y()*weight, hx.z()));*/
        Pose3 res = p3.between(hx, H3);
        
        return measured_.localCoordinates(res);
    }
};

#endif /* SRC_OPTIMIZATION_BETWEENTHREE_H_ */
