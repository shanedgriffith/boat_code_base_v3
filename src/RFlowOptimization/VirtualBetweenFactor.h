/*
 * VirtualBetweenFactor.h
 *
 *  Created on: Jan 19, 2017
 *      Author: shane
 */



#ifndef SRC_RFLOWOPTIMIZATION_VIRTUALBETWEENFACTOR_H_
#define SRC_RFLOWOPTIMIZATION_VIRTUALBETWEENFACTOR_H_

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

    gtsam::Pose3 _p0;
    gtsam::Pose3 _p1;
public:
    typedef typename boost::shared_ptr<VirtualBetweenFactor> shared_ptr;

    VirtualBetweenFactor() {}

    //the key for a between factor is just the label that identifies it
    //specify 1, 2, and 3 in reverse chronological order.
    //i.e., x_t+1, x_t, and then v_t+1
    VirtualBetweenFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Pose3 p0, gtsam::Pose3 p1, const gtsam::SharedNoiseModel& model):
         Base(model, j1, j2), _p0(p0), _p1(p1){}

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
    gtsam::Vector evaluateError(const gtsam::Pose3& p1frame0, const gtsam::Pose3& p0frame1,
                         boost::optional<gtsam::Matrix&> H1frame0 = boost::none,
                         boost::optional<gtsam::Matrix&> H0frame1 = boost::none) const
    {
        //p0f1_solve =should= p1.compose(p1.btwn(p1frame0)*p1frame0.btwn(p0)*p1frame0.btwn(p1))
        //p1f0_solve =should= p0.compose(p0.btwn(p0frame1)*p0frame1.btwn(p1)*p0frame1.btwn(p0))
        //should nearzero be the sum in both directions?
        //perhaps using zeros.localCoordinates(n1) + zeros.localCoordinates(n0);
        //hmm, that step computes the tangent vector, presumably to reach zero from nearzero.
        //but if both are used..
        //maybe use a static variable which is toggled, and the value determines which nearzero is returned.
        //may not matter.
        
        gtsam::Pose3 btwn0, btwn1, btwn2, nearzero;
        if(H1frame0) {
            gtsam::Matrix d0, d1;
            gtsam::Pose3 ptwn0, ptwn1, ptwn2;
            gtsam::Pose3 expected_p0f1, expected_p1f0;
            ptwn0 = _p0.between(p0frame1);
            ptwn1 = p0frame1.between(_p1);
            ptwn2 = p0frame1.between(_p0);
            expected_p1f0 = _p0.compose(ptwn0*ptwn1*ptwn2);
            expected_p1f0.between(p1frame0, d1, *H1frame0);
            
            btwn0 = _p1.between(p1frame0);
            btwn1 = p1frame0.between(_p0);
            btwn2 = p1frame0.between(_p1);
            expected_p0f1 = _p1.compose(btwn0*btwn1*btwn2);
            nearzero = expected_p0f1.between(p0frame1, d0, *H0frame1);
        } else {
            btwn0 = _p1.between(p1frame0);
            btwn1 = p1frame0.between(_p0);
            btwn2 = p1frame0.between(_p1);
            nearzero = _p1.compose(btwn0*btwn1*btwn2).between(p0frame1);
        }
        gtsam::Pose3 zeros = gtsam::Pose3::identity();//needed?
        return zeros.localCoordinates(nearzero);
        
/*  //working approach, though may only be an approximation
        //note. the jacobians here could be wrong.
        gtsam::Pose3 diff0, diff1, nearzero;
        if(H1frame0) {
            gtsam::Matrix dH0;
            gtsam::Matrix dH1;
            gtsam::Matrix d0, d01, d1, d10;
            diff0 = _p0.between(p0frame1, d0, d01);
            diff1 = p1frame0.between(_p1, d10, d1);
            nearzero = diff0.between(diff1, dH0, dH1);
            *H0frame1 = dH0 * d01;
            *H1frame0 = dH1 * d10;
        } else {
            diff0 = _p0.between(p0frame1);
            diff1 = p1frame0.between(_p1);
            nearzero = diff0.between(diff1);
        }
        gtsam::Pose3 zeros = gtsam::Pose3::identity();//needed?
        return zeros.localCoordinates(nearzero);*/

        //previous approach. The different formulations depending on whether the Jacobian is wanted may be problematic.
//        Pose3 diff0 = _p0.between(p0frame1);
//        Pose3 diff1 = p1frame0.between(_p1);
//        Pose3 nearzero = diff0.between(diff1);
//        if(H1frame0){
//            gtsam::Matrix dH0;
//            gtsam::Matrix dH1;
//            Pose3 top = p0frame1.compose(_p0.between(p1frame0), H0frame1).between(_p1, dH0);
//            Pose3 bot = p1frame0.between(_p1.between(p0frame1), H1frame0).between(_p0, dH1);
//            nearzero = diff0.between(diff1, dH0, dH1);
//            *H0frame1 = dH0 * (*H0frame1);
//            *H1frame0 = dH1 * (*H1frame0);
//        }
//        gtsam::Pose3 zeros = gtsam::Pose3::identity();//needed?
//        return zeros.localCoordinates(nearzero);
    }
};

#endif /* SRC_RFLOWOPTIMIZATION_VIRTUALBETWEENFACTOR_H_ */
