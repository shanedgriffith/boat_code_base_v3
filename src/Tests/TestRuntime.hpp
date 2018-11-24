


#ifndef SRC_TESTS_TESTRUNTIME_H_
#define SRC_TESTS_TESTRUNTIME_H_

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <vector>


class TestRuntime {
private:
    int FindRestart(std::string fname);
    
    struct in_progress{
        std::string date;
        int idx;
        in_progress(std::string d, int i): date(d), idx(i){}
    };
    
    std::vector<TestRuntime::in_progress> GetRunningProcesses();
    
    
public:
    std::string _origin;
    
    TestRuntime(std::string origin): _origin(origin) {}
    
    void RunningProcesses();
    
    
    
    
};

#endif /* SRC_TESTS_TESTRUNTIME_H_ */





