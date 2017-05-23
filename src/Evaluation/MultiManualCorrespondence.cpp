/*
 * MultiManualCorrespondence.cpp
 *
 *  Created on: Nov 18, 2016
 *      Author: shane
 */

#include "MultiManualCorrespondence.hpp"


int MultiManualCorrespondence::GetImageNumberFromImagePath(std::string imagepath)
{
    int l = imagepath.rfind(".");
    int s = imagepath.rfind("/");
    int f = imagepath.rfind("/", s-1);
    std::string imgno =imagepath.substr(s+1,l-s-1);
    std::string mil = imagepath.substr(f+1, s-f-1);
    return stod(mil)*1000+stod(imgno);
}

std::string MultiManualCorrespondence::GetDateFromImagePath(std::string imagepath)
{
    int e = imagepath.rfind("/");
    e = imagepath.rfind("/", e-1);
    int s = imagepath.rfind("/", e-1);
    return imagepath.substr(s+1, e-1);
}

void MultiManualCorrespondence::ProcessLineEntries(int type, std::vector<std::string> lp){
    if(lp[0].at(0)=='%')return;
    if(lp[0].at(0)=='F')return;
    if(lp.size()<11)return;
    int cnum = stod(lp[4]);
    if(cnum == 0){
    	handlabels hl = {
    			GetDateFromImagePath(lp[2]),
				GetDateFromImagePath(lp[3]),
				GetImageNumberFromImagePath(lp[2]),
				GetImageNumberFromImagePath(lp[3]),
				stod(lp[10])
    	};
    	correspondenceset.push_back(hl);
    }
    std::vector<correspondence> p(10);
    printf("processed line\n");
    correspondenceset[correspondenceset.size()-1].points.push_back({cv::Point2f(stod(lp[6]), stod(lp[7])), cv::Point2f(stod(lp[8]), stod(lp[9]))});
}

void MultiManualCorrespondence::ReadDelimitedFile(std::string file, int type){
    FILE * fp = OpenFile(file,"r");
    char line[LINESIZE]="";

    while (fgets(line, LINESIZE-1, fp))
    {
    	printf("line: %s\n", line);
        char * tmp = line;
        std::vector<std::string> lp = ParseLine(tmp);
        ProcessLineEntries(type, lp);
    }
    fclose(fp);
}

int MultiManualCorrespondence::GetNumberOfImagePairs(){
	return correspondenceset.size();
}

void MultiManualCorrespondence::PrintHist(){
	std::vector<int> numpoints(20, 0);
	for(int i=0; i<correspondenceset.size(); i++){
		numpoints[correspondenceset[i].points.size()]++;
	}

	for(int i=0; i<numpoints.size(); i++){
		std::cout << i << ": " << numpoints[i] << std::endl;
	}
}

int MultiManualCorrespondence::MedianNumberOfPointsPerPair(){
	//Find the median in O(n) time using radix sort.
	std::vector<int> numpoints(20, 0);
	for(int i=0; i<correspondenceset.size(); i++){
		numpoints[correspondenceset[i].points.size()]++;
	}
	int count=0;
	int i=0;
	for(; count < correspondenceset.size()/2; i++)
		count += numpoints[i];
	return i-1;
}



