/*
 * RFlowFactorGraph.hpp
 *
 *  Created on: Jul 28, 2016
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_RFLOWFACTORGRAPH_HPP_
#define SRC_RFLOWOPTIMIZATION_RFLOWFACTORGRAPH_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Symbol.h>

#include "Optimization/FactorGraph.hpp"

class RFlowFactorGraph: public FactorGraph {
private:
	gtsam::noiseModel::Isotropic::shared_ptr pointNoise;
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise0;
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise1;
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise0;
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise1;
    gtsam::noiseModel::Diagonal::shared_ptr poseNoiseP;

	void InitializeNoiseModels();

	gtsam::Pose3 VectorToPose(std::vector<double>& p);

	std::vector<int> lastcnums;
	std::vector<std::unordered_map<int, int> > pnum_to_ckey;
	std::unordered_map<int, int> surveycnum;
public:
	RFlowFactorGraph(): FactorGraph(){
	    InitializeNoiseModels();
	    sppf_prune = false; //keep all the landmark tracks.
	}

	bool AddPose(gtsam::Symbol s, gtsam::Pose3 p);
	bool AddPose(int survey, int pnum, gtsam::Pose3 p);
	void AddVirtualBTWNFactor(int survey0, int pnum0, int survey1, int pnum1, gtsam::Pose3 p0, gtsam::Pose3 p1, double val=0.001);
	void AddLocalizationFactors(gtsam::Cal3_S2::shared_ptr k, int survey, int pnum, std::vector<gtsam::Point3>& p3d, std::vector<gtsam::Point2>& p2d, std::vector<double>& inliers);
	void AddBTWNFactor(int survey0, int pnum0, int survey1, int pnum1, gtsam::Pose3 odom, bool tight=false);
    void AddCustomBTWNFactor(int survey0, int pnum0, int survey1, int pnum1, gtsam::Pose3 odom, double val);

	virtual gtsam::Symbol GetSymbol(int survey, int pnum);
	bool VariableExists(int survey, int pnum);
	void Clear();
};



#endif /* SRC_RFLOWOPTIMIZATION_RFLOWFACTORGRAPH_HPP_ */
