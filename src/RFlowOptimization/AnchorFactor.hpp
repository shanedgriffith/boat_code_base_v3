/*
 * AnchorFactor.hpp
 *
 *  Created on: Sep 8, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_ANCHORFACTOR_HPP_
#define SRC_RFLOWOPTIMIZATION_ANCHORFACTOR_HPP_


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
class AnchorFactor: public gtsam::NoiseModelFactor4<Pose3, Pose3, Pose3, Pose3>{
public:
    typedef Pose3 T;
private:
    typedef AnchorFactor This;
    typedef gtsam::NoiseModelFactor4<Pose3, Pose3, Pose3, Pose3> Base;
    
    Pose3 _tf_p0_to_p1frame0;
public:
    typedef typename boost::shared_ptr<AnchorFactor> shared_ptr;
    
    AnchorFactor() {}
    
    //the key for a between factor is just the label that identifies it
    //specify 1, 2, and 3 in reverse chronological order.
    //i.e., x_t+1, x_t, and then v_t+1
    AnchorFactor(gtsam::Key a0, gtsam::Key a1, gtsam::Key p0, gtsam::Key p1, gtsam::Pose3 tf_p0_to_p1frame0, const gtsam::SharedNoiseModel& model):
    Base(model, a0, a1, p0, p1), _tf_p0_to_p1frame0(tf_p0_to_p1frame0){}
    
    virtual ~AnchorFactor() {}
    
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(gtsam::NonlinearFactor::shared_ptr(new This(*this))); }
    
    virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "AnchorFactor("
        << keyFormatter(this->key1()) << ","
        << keyFormatter(this->key2()) << ","
        << keyFormatter(this->key3()) << ","
        << keyFormatter(this->key4()) << ")\n";
        this->noiseModel_->print("  noise model: ");
    }
    
    /*On jacobians:
     * https://studywolf.wordpress.com/2013/09/02/robot-control-jacobians-velocity-and-force/
     * http://robotics.itee.uq.edu.au/~metr4202/2013/tpl/Chapter%204%20-%20Jacobain%20-%20from%20Khatib%20-%20Introduction%20to%20Robotics.pdf
     * */
    gtsam::Vector evaluateError(const Pose3& a0, const Pose3& a1, const Pose3& p0, const Pose3& p1,
                                boost::optional<gtsam::Matrix&> Ha0 = boost::none,
                                boost::optional<gtsam::Matrix&> Ha1 = boost::none,
                                boost::optional<gtsam::Matrix&> Hp0 = boost::none,
                                boost::optional<gtsam::Matrix&> Hp1 = boost::none) const
    {
        Pose3 nearzero;
        if(Ha0) {
            //maybe the gradient could be computed for each one separately to simplify computing H?
            //presumably, although the min shifts with the changing variable values, the cost field
            //and the trajectory towards the min are similar over the iterations.
            
            Pose3 est_a0 = a1.compose(p1).compose(_tf_p0_to_p1frame0.inverse()).compose(p0.inverse());
            a0.between(est_a0, *Ha0);
            
            Pose3 est_a1 = a0.compose(p0).compose(_tf_p0_to_p1frame0).compose(p1.inverse());
            a1.between(est_a1, *Ha1);
            
            Pose3 est_p0 = a0.inverse().compose(a1).compose(p1).compose(_tf_p0_to_p1frame0.inverse());
            p0.between(est_p0, *Hp0);
            
            Pose3 est_p1 = a1.inverse().compose(a0).compose(p0).compose(_tf_p0_to_p1frame0);
            nearzero = p1.between(est_p1, *Hp1);
        } else {
            Pose3 est_p1 = a1.inverse().compose(a0).compose(p0).compose(_tf_p0_to_p1frame0);
            nearzero = p1.between(est_p1);
        }
        gtsam::Pose3 zeros = gtsam::Pose3::identity();
        return zeros.localCoordinates(nearzero);
    }
};


#endif /* SRC_RFLOWOPTIMIZATION_ANCHORFACTOR_HPP_ */
