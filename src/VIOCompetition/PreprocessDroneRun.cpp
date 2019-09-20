#include "PreprocessDroneRun.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ParseDroneRun.hpp"
#include <VisualOdometry/VisualOdometry.hpp>
#include <VisualOdometry/KLT.hpp>
#include <Visualizations/IMDraw.hpp>
#include <Visualizations/SLAMDraw.h>


using namespace std;

void
PreprocessDroneRun::ProcessLineEntries(int type, vector<string>& lp)
{
    if(lp[0].compare("#") == 0) return;
    timestamps.push_back(stod(lp[1]));
    paths.push_back(lp[2]);
}

std::vector<std::string>
PreprocessDroneRun::ParseLine(char * line)
{
    return ParseLineAdv(line, " ");
}

void
PreprocessDroneRun::ReadDelimitedFile(string file, int type)
{
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE];
    
    while (fgets(line, LINESIZE-1, fp)) {
        char * tmp = line;
        vector<string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

void
PreprocessDroneRun::ProcessRawVideo()
{
    Camera cam = ParseDroneRun::GetCamera();
    string imgbase = _base + _name + "/";
    
    KLT k(cam);
    std::vector<double> KLTparms = {12, 20., 200., 4., 100., 100., 100., 9., 125., 40.};
    k.SetSizes(KLTparms);
    
    for(int i=0; i<timestamps.size(); i++)
    {
        std::string imgpath = imgbase + paths[i];
        cv::Mat image = cv::imread(imgpath);
        
        double imtime = timestamps[i];
        
        ParseFeatureTrackFile PFT = k.TrackKLTFeatures(image, _base + _name, i, imtime);
        PFT.WriteFeatureTrackFile();
        
        std::cout << "saving pft as " << PFT.siftfile << std::endl;
    }
}

void
PreprocessDroneRun::FindKLTParams()
{
    bool show = true;
    if(show) cv::namedWindow("klt points");
    
    Camera cam = ParseDroneRun::GetCamera();
    string imgbase = _base + _name + "/";
    
    KLT k(cam);
    //long tracks, but they get pulled along by occlusions, and aren't spread out well.
    //std::vector<double> parms = {30., 45., 600., 20., 400., 400., 100., 9., 125., 40.};
    
    //features aren't as susceptible to dragging, but they still do, and they're not well spread throughout the scene.
    //std::vector<double> parms = {30., 45., 600., 20., 30., 30., 100., 9., 125., 40.};
    
    //works fairly well with the smaller grid size in reducing feature wiping by occluders, but can't handle larger transitions.
    //some points seem to drift.
    //std::vector<double> parms = {18., 36., 850., 4., 30., 30., 100., 9., 125., 40.};
    std::vector<double> KLTparms = {12., 20., 200., 4., 100., 100., 100., 9., 125., 40.};
    k.SetSizes(KLTparms);
    for(int i=0; i<KLTparms.size(); i++)
    {
        std::cout << KLTparms[i] << ", ";
    } std::cout << std::endl;
    
    int m_track = 5;
    double sumall=0;
    int count = 0;
    int start = 234;
    int counter = 0;
    for(int i=0; i<timestamps.size(); i++)
    {
        std::string imgpath = imgbase + paths[i];
        cv::Mat image = cv::imread(imgpath);
        if(image.rows == 0)
        {
            std::cout << "couldn't retrieve the image." << std::endl;
            exit(-1);
        }
        counter++;
        if(counter < start) continue;
        std::cout << std::endl;
        
        ParseFeatureTrackFile PFT = k.TrackKLTFeatures(image, _base + _name, 0, 0);
        vector<LandmarkTrack> inactive = PFT.ProcessNewPoints((int) 'x', counter, active);
        if(inactive.size() > 0)
        {
            int suml = 0;
            int minl = 10000000;
            int maxl = 0;
            int gttrack = 0;
            for(int i=0; i<inactive.size(); i++) {
                suml += inactive[i].points.size();
                minl = std::min(minl, (int) inactive[i].points.size());
                maxl = std::max(maxl, (int) inactive[i].points.size());
                gttrack = (inactive[i].points.size()>m_track)?gttrack+1:gttrack;
            }
            sumall += suml;
            count += inactive.size();
            std::cout << "KLT track "<<counter<<": " << 1.0*suml/inactive.size() << ", [" << minl << ", " << maxl << "] all avg: " << 1.0*sumall/count << ", num > " << gttrack << " of " << inactive.size() << std::endl;
        }
        
        if(show)
        {
            IMDraw art(image);
            for(int j=0; j<PFT.ids.size(); j++)
            {
                art.DrawPoint(PFT.imagecoord[j].x(), PFT.imagecoord[j].y(), PFT.ids[j]);
            }
            cv::imshow("klt points", image);
            char c = cvWaitKey(30);
            if(c == 'q') exit(1);
        }
    }
    
    if(show) cv::destroyWindow("klt points");
}






































































