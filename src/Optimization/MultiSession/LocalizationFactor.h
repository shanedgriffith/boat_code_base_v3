/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationFactor.h (was ProjectionFactor.h)
 * @brief Basic bearing factor from 2D measurement
 * @author Chris Beall
 * @author Richard Roberts
 * @author Frank Dellaert
 * @author Alex Cunningham
 * @author Shane Griffith (small modifications)
 */

#ifndef SRC_RFLOWOPTIMIZATION_LOCALIZATIONFACTOR_H_
#define SRC_RFLOWOPTIMIZATION_LOCALIZATIONFACTOR_H_

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/optional.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>

/**
* Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
* i.e. the main building block for visual SLAM.
* @addtogroup SLAM
*/

template<class POSE=gtsam::Pose3, class CALIBRATION=gtsam::Cal3_S2>
class LocalizationFactor: public gtsam::NoiseModelFactor1<POSE> {
public:
    typedef POSE T;
    typedef CALIBRATION C;
protected:

    // Keep a copy of measurement and calibration for I/O
    gtsam::Point2 measured_;                    ///< 2D measurement
    gtsam::Point3 world_;			////< 3D point
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
    boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:

    /// shorthand for base class type
    typedef gtsam::NoiseModelFactor1<POSE> Base;

    /// shorthand for this class
    typedef LocalizationFactor<POSE, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
    LocalizationFactor() : throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param K shared pointer to the constant calibration
     * @param body_P_sensor is the transform from body to sensor frame (default identity)
     */
    LocalizationFactor(const gtsam::Point2& measured, const gtsam::Point3& world, const gtsam::SharedNoiseModel& model,
        gtsam::Key poseKey, const boost::shared_ptr<CALIBRATION>& K,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, poseKey), measured_(measured), world_(world), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     * @param body_P_sensor is the transform from body to sensor frame  (default identity)
     */
    LocalizationFactor(const gtsam::Point2& measured, const gtsam::Point3& world, const gtsam::SharedNoiseModel& model,
        gtsam::Key poseKey, const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality,
        boost::optional<POSE> body_P_sensor = boost::none) :
          Base(model, poseKey), measured_(measured), world_(world), K_(K), body_P_sensor_(body_P_sensor),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~LocalizationFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
      std::cout << s << "LocalizationFactor, z = ";
      measured_.print();
      world_.print();
      if(this->body_P_sensor_)
        this->body_P_sensor_->print("  sensor pose in body frame: ");
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const gtsam::NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && this->measured_.equals(e->measured_, tol)
          && this->world_.equals(e->world_, tol)
          && this->K_->equals(*e->K_, tol)
          && ((!body_P_sensor_ && !e->body_P_sensor_) || (body_P_sensor_ && e->body_P_sensor_ && body_P_sensor_->equals(*e->body_P_sensor_)));
    }

    /// Evaluate error h(x)-z and optionally derivatives
    //virtual Vector evaluateError(const X& x, boost::optional<Matrix&> H = boost::none) const = 0;
    gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H1 = boost::none) const {
      try {
        if(body_P_sensor_) {
          if(H1) {
            gtsam::Matrix H0;
            gtsam::PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
            gtsam::Point2 reprojectionError(camera.project(world_, H1) - measured_);
            *H1 = *H1 * H0;
            return reprojectionError.vector();
          } else {
            gtsam::PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
            gtsam::Point2 reprojectionError(camera.project(world_, H1) - measured_);
            return reprojectionError.vector();
          }
        } else {
          gtsam::PinholeCamera<CALIBRATION> camera(pose, *K_);
          gtsam::Point2 reprojectionError(camera.project(world_, H1) - measured_);
          return reprojectionError.vector();
        }
      } catch( gtsam::CheiralityException& e) {
        if (H1) *H1 = gtsam::zeros(2,6);
//        if (verboseCheirality_)
//          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
//              " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return gtsam::ones(2) * 2.0 * K_->fx();
    }

    /** return the measurement */
    const gtsam::Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(world_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
};

#endif /* SRC_RFLOWOPTIMIZATION_LOCALIZATIONFACTOR_H_ */
