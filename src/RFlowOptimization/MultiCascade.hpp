/*
 * MultiSessionIterativeSmoothingAndRefinement.hpp
 *
 *  Created on: Oct 11, 2017
 *      Author: shane
 */

#ifndef SRC_RFLOWOPTIMIZATION_MULTICASCADE_HPP_
#define SRC_RFLOWOPTIMIZATION_MULTICASCADE_HPP_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>
#include <unordered_map>

#include <ImageAlignment/FlowFrameworks/MachineManager.h>

#include "MultiSessionOptimization.hpp"
#include "OptimizationMachine.hpp"
#include "EvaluateRFlow.hpp"

class MultiCascade: public MultiSessionOptimization {
protected:
    const int MAX_CASC_ITERS = 5;
    
    std::vector<std::vector<std::vector<double> > > landmarks;
    std::vector<std::vector<std::vector<double> > > poses;
    std::vector<int> runset;
    std::vector<ParseOptimizationResults> originPOR;
    
    std::vector<std::vector<double>> isc_rerror;
    std::vector<std::vector<std::vector<int> > > forwardLMap;
    
    void BuildLandmarkSet();
    
    std::vector<bool> LPDInlierTest(int s, int l, double LPD_RERROR_THRESHOLD, std::vector<double>& error);
    int EvaluateLPD(std::vector<std::vector<std::vector<double> > >& poseresult, std::vector<std::vector<std::vector<double> > >& landmarks, int s, int j, std::vector<EvaluateRFlow*> perf, std::vector<std::unordered_map<int, double> >& errorcache);
    double UpdateErrorIterative();
    
    void SaveResults();
    
    void RunIteration(bool firstiter);
    
    MachineManager man;
    std::vector<OptimizationMachine*> ws;
public:
    std::string _origin_dir;
    
    MultiCascade(Camera& cam, std::string results_dir, std::string pftbase, std::string date = "", int nthreads=8);
    
    ~MultiCascade(){
        man.WaitForMachine(true);
        for(int i=0; i<ws.size(); i++)
            delete(ws[i]);
    }
    
    void IterativeMerge();
};


#endif /* SRC_RFLOWOPTIMIZATION_MULTICASCADE_HPP_ */
