/*
 * VirtualBetweenFactor.h
 *
 *  Created on: Jan 19, 2017
 *      Author: shane
 */



#ifndef __BundleAdjustOneDataset__VirtualBetweenFactor__
#define __BundleAdjustOneDataset__VirtualBetweenFactor__

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
public:
    typedef POSE T;
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
        return zeros.localCoordinates(nearzero);
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

#endif /* defined(__BundleAdjustOneDataset__VirtualBetweenFactor__) */
