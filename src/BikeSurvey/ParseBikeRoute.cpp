
#include "ParseBikeRoute.hpp"

using namespace std;

void ParseBikeRoute::ProcessLineEntries(int type, vector<string>& lp){
    if(lp.size()<8)return;
    timings.push_back(stod(lp[0]));
    vector<double> p = {stod(lp[1]), stod(lp[2]), stod(lp[3]), stod(lp[4]), stod(lp[5]), stod(lp[6])};
    poses.push_back(p);
    omega.push_back(stod(lp[7]));
}

void ParseBikeRoute::ReadDelimitedFile(string file, int type) {
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

int ParseBikeRoute::GetImageNumber(int auxidx){
    return auxidx;
}

double ParseBikeRoute::GetAvgAngularVelocity(int sidx, int eidx){
    /* The average angular velocity between two indices [sidx, eidx].
     * */
    sidx = max(0,sidx);
    eidx = min((int)omega.size()-1, eidx);
    double sum = 0.0;
    for(int i=sidx; i<=eidx; i++) {
        sum += omega[i];
    }
    if(eidx-sidx==0) return omega[eidx];
    return sum/(eidx-sidx);
}

Camera ParseBikeRoute::GetCamera(){
    Camera nexus(1206.41699, 1205.09164, 636.766777, 371.147712, 1280, 720);
    nexus.SetDistortion(0.0817643033, 0.682168738, 0.00118562419, -0.00100627619, 0.0); //assume k3 is unused???
    //0.0817643033, 0.682168738, 0.00118562419, -0.00100627619, -7.05688254
    return nexus;
}

vector<double> ParseBikeRoute::GetDrawScale(){
    return {-2000,400,-500,1500};
}













































