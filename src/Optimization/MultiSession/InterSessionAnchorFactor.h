#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


//an inter--session factor when two sessions are optimized together.
template <class POSE>
class InterSessionAnchorFactor : gtsam::NoiseModelFactor3<POSE, POSE, POSE>
{
private:
    typedef InterSessionAnchorFactor This;
    typedef gtsam::NoiseModelFactor3<POSE, POSE, POSE> Base;
    
    gtsam::Pose3 hat_pj_a_to_pkj_b;
    
public:
    
    InterSessionAnchorFactor() {}
    
    InterSessionAnchorFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Pose3 loop_closure, const gtsam::SharedNoiseModel& model):
    Base(model, j1, j2, j3), loop_closure_(loop_closure) {}
    
    virtual ~InterSessionAnchorFactor() {}
    
    gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                                                                  gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "BetweenFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ",";
        << keyFormatter(this->key3()) << ")\n";
        this->noiseModel_->print("  noise model: ");
    }
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pj_a, const gtsam::Pose3& pk_b, const gtsam::Pose3& pkj,
                                boost::optional<gtsam::Matrix&> Hja = boost::none,
                                boost::optional<gtsam::Matrix&> Hkb = boost::none,
                                boost::optional<gtsam::Matrix&> HT = boost::none) const
    {
        gtsam::Pose3 matchja_kjb, matchkb_kjb, nearzero;
        if(H0) {
            gtsam::Matrix dHj, dHk, dH;
            gtsam::Matrix dj, dk;
            
            matchja_kjb = pj_a.compose(hat_pj_a_to_pkj_b, dHj);
            matchkb_kjb = pk_b.compose(pkj, dHk, dHT);
            nearzero = matchkb_kjb.between(matchja_kjb, dk, dj);
            
            *Hja = dj * dHj;
            *Hkb = dk * dHk;
            *HT  = dk * dHT;
        } else {
            matchja_kjb = pj_a.compose(hat_pj_a_to_pkj_b);
            matchkb_kjb = pk_b.compose(pkj);
            nearzero = matchkb_kjb.between(matchja_kjb);
        }
        
        gtsam::Pose3 zeros = gtsam::Pose3::identity();
        return zeros.localCoordinates(nearzero);
        
    }
    
    
    gtsam::Vector evaluateError(const gtsam::Pose3& pk_b, const gtsam::Pose3& pkj,
                                boost::optional<gtsam::Matrix&> Hkb = boost::none,
                                boost::optional<gtsam::Matrix&> HT = boost::none) const
    {
        /*
            hat_pkj_b = pj_a.compose(hat_pj_a_to_pkj_b) //pre-computed, pre-supplied.
         */
        
        gtsam::Pose3 matchkb_kjb, nearzero;
        if(H0) {
            gtsam::Matrix dHk, dH;
            gtsam::Matrix dk;
            
            matchkb_kjb = pk_b.compose(pkj, dHk, dHT);
            nearzero = matchkb_kjb.between(hat_pkj_b, dk);
            
            *Hkb = dk * dHk;
            *HT  = dk * dHT;
        } else {
            matchkb_kjb = pk_b.compose(pkj);
            nearzero = matchkb_kjb.between(hat_pkj_b);
        }
        
        gtsam::Pose3 zeros = gtsam::Pose3::identity();
        return zeros.localCoordinates(nearzero);
        
    }
}
























