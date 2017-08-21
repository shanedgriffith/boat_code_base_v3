/*
 * AnchorISCFactor.h
 *
 *  Created on: Aug 14, 2017
 *      Author: shane
 */


#ifndef SRC_RFLOWOPTIMIZATION_ANCHORISCFACTOR_H_
#define SRC_RFLOWOPTIMIZATION_ANCHORISCFACTOR_H_

#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>


class AnchorISCFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>{
private:
    typedef AnchorISCFactor This;
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> Base;
    
//    gtsam::Pose3 _p0;
//    gtsam::Pose3 _p1;
//    gtsam::Pose3 _isc;
    gtsam::Pose3 _p0ISCp1I;
    gtsam::Pose3 _p0ISCp1I_inv;
public:
    typedef typename boost::shared_ptr<AnchorISCFactor> shared_ptr;
    
    AnchorISCFactor() {}
    
    AnchorISCFactor(gtsam::Key a1, gtsam::Key a2, gtsam::Pose3 p0, gtsam::Pose3 p1, gtsam::Pose3 isc, const gtsam::SharedNoiseModel& model):
    Base(model, a1, a2), {
        _p0ISCp1I = p0.compose(isc).compose(p1.inverse());
        _p0ISCp1I_inv = _p0ISCp1I.inverse();
    }
    
    virtual ~AnchorISCFactor() {}
    
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
    gtsam::Vector evaluateError(const gtsam::Pose3& anchor0, const gtsam::Pose3& anchor1,
                                boost::optional<gtsam::Matrix&> Ha0 = boost::none,
                                boost::optional<gtsam::Matrix&> Ha1 = boost::none) const
    {
        //gtsam::Pose3 p1f0_est = anchor0.compose(_p0).between(anchor1.compose(_p1));
        //gtsam::Pose3 nearzero = p1f0_est.between(_isc);
        //this would be if _isc was p1f0 and is what was to be adjusted.
        //here isc is tf_p0_to_p1frame0
        //perhaps: a1p1=a0p0ISC
        //         a1p1p1^'1=a0p0ISCp1^'1
        //         a1=a0p0ISCp1^'1
        //and
        //          a1p1=a0p0ISC
        //          a1p1ISC^'1=a0p0
        //          a1p1ISC^'1p0^'1=a0
        //so
        //          p1ISC^'1p0^'1 =?= (p0ISCp1^'1)^-1   (I think so) (AB)^-1 = B^-1A^-1
        //ok, so one constant.
        //      i.e., solve for the transform from a0 to a1, and then compute the between.
        gtsam::Pose3 nearzero;
        if(Ha0) {
            gtsam::Matrix d0, d1, d2, d3;
            nearzero = anchor0.compose(_p0ISCp1I, d0, d1).between(anchor1, d2, d3);
            *Ha0 = d2 * d0;
            anchor1.compose(_p0ISCp1I_inv, d0, d1).between(anchor0, d2, d3);
            *Ha1 = d2 * d0;
        } else {
            nearzero = anchor0.compose(_p0ISCp1I).between(anchor1);
        }
        gtsam::Pose3 zeros = gtsam::Pose3::identity();
        return zeros.localCoordinates(nearzero);
    }
};

#endif /* SRC_RFLOWOPTIMIZATION_ANCHORISCFACTOR_H_ */
