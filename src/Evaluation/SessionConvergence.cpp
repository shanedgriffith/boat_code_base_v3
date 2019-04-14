



#include "SessionConvergence.hpp"
#include "Optimization/SingleSession/GTSAMInterface.h"

using namespace std;

double SessionConvergence::MeasureREDifference(std::vector<double>& bref, std::vector<double>& bcur, std::vector<gtsam::Point3>& p3dref, std::vector<gtsam::Point3>& p3dcur){
    
    gtsam::Pose3 tfref = GTSAMInterface::VectorToPose(bref);
    gtsam::Pose3 tfcur = GTSAMInterface::VectorToPose(bcur);
    
    double total_error = 0;
    double count = 0;
    for(int j=0; j<p3dref.size(); j++) {
        if(p3dref[j].x()==0.0 && p3dref[j].y()==0.0 && p3dref[j].z()==0.0) continue;
        if(p3dcur[j].x()==0.0 && p3dcur[j].y()==0.0 && p3dcur[j].z()==0.0) continue;
        
        gtsam::Point3 resref = tfref.transform_to(p3dref[j]);
        gtsam::Point2 twodimref = _cam.ProjectToImage(resref);
        if(!_cam.InsideImage(twodimref)) continue; //different.
        
        gtsam::Point3 rescur = tfcur.transform_to(p3dcur[j]);
        gtsam::Point2 twodimcur = _cam.ProjectToImage(rescur);
        if(!_cam.InsideImage(twodimcur)) continue; //different.
        
        double err = twodimcur.distance(twodimref);
        total_error += err;
        count++;
    }

    return total_error/count;
}

void SessionConvergence::CompareSessions(){
    string optbase = "/home/shaneg/results/optconvergence/";
    std::vector<std::string> optset = FileParsing::ListDirsInDir(optbase);
    
    //use the last one as the reference.
    vector<ParseOptimizationResults> ref;
    vector<ReprojectionFlow*> rf;
    vector<Map*> maps;
    string refsetdir = optbase + optset[optset.size()-1] + "/";
    std::vector<std::string> optref = FileParsing::ListDirsInDir(refsetdir);
    for(int i=0; i<optref.size(); i++) {
        ParseOptimizationResults POR(refsetdir, optref[i]);
        ref.push_back(POR);
        maps.push_back(new Map(refsetdir));
        maps[i]->LoadMap(optref[i]);
        rf.push_back(new ReprojectionFlow(_cam, *maps[i]));
    }
    
    vector<vector<vector<double> > > res(optset.size(), vector<vector<double> >());
    
    for(int i=0; i<optset.size(); i++) {
        string cursetdir = optbase + optset[i] + "/";
        std::vector<std::string> optcur = FileParsing::ListDirsInDir(cursetdir);
        ParseOptimizationResults PORzero(cursetdir, optcur[0]);
        vector<vector<double> > resset(optcur.size()-1, vector<double>());
        for(int j=1; j<optcur.size(); j++) {
            ParseOptimizationResults POR(cursetdir, optcur[j]);
            //compute RE for each pose wrt the nearest one of session 0
            std::vector<double> stats = {0,0};
            std::vector<double> errs;
            
            for(int k=0; k<POR.boat.size(); k++) {
                double g=0;
                int pidx = rf[j]->IdentifyClosestPose(ref[0].boat, ref[j].boat[k], &g, false);
                //3D points for the less converged map
                ParseFeatureTrackFile pftf = ParseFeatureTrackFile::LoadFTF(_cam, _pftbase + optcur[j], ref[j].ftfilenos[k]);
                std::vector<gtsam::Point3> p3dcur = POR.GetSubsetOf3DPoints(pftf.ids);
                std::vector<gtsam::Point3> p3dref = ref[j].GetSubsetOf3DPoints(pftf.ids);
                vector<double> boatcur = PORzero.boat[pidx];
                vector<double> boatref = ref[0].boat[pidx];
                
                double re = MeasureREDifference(boatref, boatcur, p3dref, p3dcur);
                errs.push_back(re);
                stats[0] += re;
            }
            stats[0] /= errs.size();
            
            //std
            for(int k=0; k<errs.size(); k++)
                stats[1] += pow(errs[k] - stats[0], 2);
            stats[1] = pow(stats[1]/errs.size(), 0.5);
            resset.push_back(stats);
        }
        res.push_back(resset);
    }
    
    for(int i=0; i<optset.size()-1; i++){
        std::cout << optref[i] << ": ";
        for(int j=0; j<optset.size(); j++){
            std::cout << res[i][j][0] <<"("<<res[i][j][0]<<")\t";
        }std::cout << std::endl;
    }
}













































































































