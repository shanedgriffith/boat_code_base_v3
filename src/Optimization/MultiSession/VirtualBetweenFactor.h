#pragma once

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
class VirtualBetweenFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>{
private:
    typedef VirtualBetweenFactor This;
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;

    gtsam::Pose3 _pj_a;
    gtsam::Pose3 _pk_b;
public:
//    typedef typename boost::shared_ptr<VirtualBetweenFactor> shared_ptr;

    VirtualBetweenFactor() {}

    //the key for a between factor is just the label that identifies it
    //specify 1, 2, and 3 in reverse chronological order.
    //i.e., x_t+1, x_t, and then v_t+1
    VirtualBetweenFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Pose3 pj_a, gtsam::Pose3 pk_b, const gtsam::SharedNoiseModel& model):
         Base(model, j1, j2), _pj_a(pj_a), _pk_b(pk_b){}

    virtual ~VirtualBetweenFactor() {}

    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "BetweenFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ")\n";
        this->noiseModel_->print("  noise model: ");
    }

    /*On jacobians:
     * https://studywolf.wordpress.com/2013/09/02/robot-control-jacobians-velocity-and-force/
     * http://robotics.itee.uq.edu.au/~metr4202/2013/tpl/Chapter%204%20-%20Jacobain%20-%20from%20Khatib%20-%20Introduction%20to%20Robotics.pdf
     * */
    gtsam::Vector evaluateError(const gtsam::Pose3& pkj_b, const gtsam::Pose3& pjk_a,
                         boost::optional<gtsam::Matrix&> Hkj_b = boost::none,
                         boost::optional<gtsam::Matrix&> Hjk_a = boost::none) const
    {
        gtsam::Pose3 j_to_k, k_to_j, nearzero;
        if(Hkj_b)
        {
            gtsam::Matrix dHjk, dHkj;
            gtsam::Matrix dj, dk, dn;
            j_to_k = _pj_a.between(pjk_a, dn, dj);
            k_to_j = _pk_b.between(pkj_b, dn, dk);
            nearzero = j_to_k.compose(k_to_j, dHjk, dHkj);
            
            *Hkj_b = dHkj * dk;
            *Hjk_a = dHjk * dj;
        }
        else
        {
            j_to_k = _pj_a.between(pjk_a);
            k_to_j = _pk_b.between(pkj_b);
            nearzero = j_to_k.compose(k_to_j);
        }
        
        gtsam::Pose3 zeros = gtsam::Pose3::identity();
        return zeros.localCoordinates(nearzero);
    }
};
