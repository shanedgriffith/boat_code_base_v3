#pragma once


#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


//an inter--session factor when two sessions are optimized together.
class OneSessionAnchor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
private:
    typedef OneSessionAnchor This;
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;
    
    gtsam::Pose3 hat_pkj_b_;
    
public:
    
    OneSessionAnchor() {}
    
    /*
     hat_pkj_b = pj_a.compose(hat_pj_a_to_pkj_b) //pre-computed, pre-supplied.
     */
    OneSessionAnchor(gtsam::Key j1, gtsam::Key j2, gtsam::Pose3 hat_pkj_b, const gtsam::SharedNoiseModel& model):
    Base(model, j1, j2), hat_pkj_b_(hat_pkj_b) {}
    
    virtual ~OneSessionAnchor() {}
    
    gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                                  gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "BetweenFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ")\n";
        this->noiseModel_->print("  noise model: ");
    }
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pk_b, const gtsam::Pose3& pkj,
                                boost::optional<gtsam::Matrix&> Hkb = boost::none,
                                boost::optional<gtsam::Matrix&> HT = boost::none) const
    {
        gtsam::Pose3 matchkb_kjb, nearzero;
        if(Hkb) {
            gtsam::Matrix dHk, dHT;
            gtsam::Matrix dk;
            
            matchkb_kjb = pk_b.compose(pkj, dHk, dHT);
            nearzero = matchkb_kjb.between(hat_pkj_b_, dk);
            
            *Hkb = dk * dHk;
            *HT  = dk * dHT;
        } else {
            matchkb_kjb = pk_b.compose(pkj);
            nearzero = matchkb_kjb.between(hat_pkj_b_);
        }
        
        gtsam::Pose3 zeros = gtsam::Pose3::identity();
        return zeros.localCoordinates(nearzero);
    }
};


























