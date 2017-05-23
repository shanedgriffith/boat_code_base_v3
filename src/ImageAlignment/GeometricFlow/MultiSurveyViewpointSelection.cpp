//
//  MultiSurveyViewpointSelection.cpp
//  boat_code_base
//
//  Created by Shane Griffith on 1/23/17.
//  Copyright © 2017 shane. All rights reserved.
//

#include "MultiSurveyViewpointSelection.hpp"


#include "IndependenceTest.hpp"

using namespace std;

bool MultiSurveyViewpointSelection::WithinThreeMonths(string date1, string date2){
    int m1 = stoi(date1.substr(2, 2));
    int m2 = stoi(date2.substr(2, 2));
    int d1 = max(m1, m2) - min(m1, m2);
    int d2 = min(m1, m2) + 12-max(m1, m2);
    int dist = min(d1, d2);
    return dist <= SURVEY_TIME_COMPARE_UP_TO;//only compare surveys up to 3 months away.
}

void MultiSurveyViewpointSelection::SortPoints(std::vector<dubs>& both){
    std::qsort(&both[0], both.size(), sizeof(dubs), [](const void* a, const void* b) {
        dubs arg1 = *static_cast<const dubs*>(a);
        dubs arg2 = *static_cast<const dubs*>(b);

        if(arg1.g < arg2.g) return -1;
        if(arg1.g > arg2.g) return 1;
        return 0;
    });
}

bool MultiSurveyViewpointSelection::IsSameScene(double gstatistic){
    /*Use the test of independence using the g-statistic to determine whether to proceed.
     * The g-statistic approximates the chi-squared distribution. If the X^2 distribution is
     * less than 0.5 then the null hypothesis ...
     * */
    //chi squared distribution with 1 dof.
    double pval = IndependenceTest::p_value(gstatistic, 1);
    bool ret = pval < 0.05;
    return ret;
}

std::vector<std::vector<double> > MultiSurveyViewpointSelection::TopKViewpoints(std::vector<ReprojectionFlow*>& rf, std::vector<std::vector<std::vector<double> > *>& poselists,
                                                                  std::vector<string>& dates, std::vector<double>& pref, int k){
    //Assumes the reference survey is the last one in the list. Thus the loop only goes to rf.size()-1.
    int nsurveys = rf.size();
    std::vector<double> gstat(nsurveys, -1);
    std::vector<int> pidx(nsurveys, -1);
    std::vector<int> snum(nsurveys, 0);
    
    for(int i=0; i<nsurveys; i++){
        snum[i] = i;
        if(!WithinThreeMonths(dates[i], dates[dates.size()-1])) continue;
        
        int tidx = man.GetOpenMachine();
        ws[tidx]->Setup(rf[i], poselists[i], pref, &gstat[i], &pidx[i]);
        if(nsurveys==1) ws[tidx]->Run();
        else man.RunMachine(tidx);
    }
    man.WaitForMachine(true);
    
    std::vector<dubs> both(nsurveys);
    for(int i=0; i<nsurveys; i++){
        both[i].g = gstat[i];
        both[i].idx = i;
    }
    SortPoints(both);

    //return the list.
    std::vector<std::vector<double> > topk;
    for(int i=nsurveys-1; topk.size()<k && i>=0; i--){
        if(!IsSameScene(both[i].g)) break;
        if(pidx[both[i].idx]==-1) break;
        std::vector<double> kthres = {(double)both[i].idx, (double) pidx[both[i].idx], both[i].g};
        topk.push_back(kthres);
    }
    return topk;
}

