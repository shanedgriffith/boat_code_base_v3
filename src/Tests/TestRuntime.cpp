
#include <iostream>

#include "FileParsing/FileParsing.hpp"
#include "TestRuntime.hpp"

using namespace std;

int TestRuntime::FindRestart(std::string fname) {
    int res=-1;
    std::ifstream fin;
    fin.open(fname);
    if(fin.is_open()) {
        fin.seekg(-2, ios_base::end);//position -2 from the end (assumes \n is the last char)
        while(fin.tellg() > 0) {
            char ch=0;
            fin.get(ch); //pos +1 from cur
            if(ch == '\n') break;
            fin.seekg(-2, ios_base::cur); //pos -2 from cur.
        }
        
        string lastLine;
        try {
            getline(fin,lastLine,',');                      // Read the current line, up to comma
            if(lastLine.length() > 6) res = stoi(lastLine);
        } catch(std::exception& e){
            std::cout << "SessionLocalization::FindRestart() " << fname << " " << lastLine << "\n"<< e.what() << std::endl;
            exit(-1);
        }
        fin.close();
    }
    return res+1;
}

std::vector<TestRuntime::in_progress> TestRuntime::GetRunningProcesses(){
    std::string base = "/home/shaneg/results/";
    
    std::vector<std::string> dirs = FileParsing::ListDirsInDir(base);
    std::vector<TestRuntime::in_progress> allinp;
    for(auto& d: dirs) {
        if(d.length() != 13 || d[0] != '1') // 6
            continue;
        
        std::string d1 = d.substr(0,6);
        
        std::vector<std::string> imnum = FileParsing::ListFilesInDir(_origin + d1 + "/images", ".jpg");
//        std::vector<std::string> imnum = FileParsing::ListFilesInDir(base + d + "/images", ".jpg");
        if(imnum.size() != 1){
            std::cout << "session format error." << d1 << ", " << imnum.size() << std::endl;
            exit(-1);
        }
        std::string imnumstr = imnum[0].substr(0, imnum[0].size()-4);
        int last = stoi(imnumstr);
        
        int ret = FindRestart(base + d + "/RFlowISC.log");
        
        if(abs(ret-last) > 20){
            TestRuntime::in_progress inp(d, ret);
            allinp.push_back(inp);
        }
    }
    return allinp;
}

void TestRuntime::RunningProcesses(){
    std::vector<in_progress> last_set;
    
    while(1) {
        int count=0;
        int countbad = 0;
        std::vector<in_progress> proc = GetRunningProcesses();
        for(auto& p : proc) {
            for(int i=0; i<last_set.size(); i++){
                if(p.date.compare(last_set[i].date) == 0){
                    if(p.idx == last_set[i].idx) {
                        std::cout << p.date << ": FROZEN." << std::endl;
                        countbad++;
                    }
                    count++;
                    break;
                }
            }
        }
        std::cout << "STATUS: " << countbad << " of " << count <<" are frozen" << std::endl;
        last_set = proc;
        sleep(5*60);
    }
}




















