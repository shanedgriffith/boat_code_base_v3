//
//  AlignmentMachine.h
//  SIFTFlow
//
//  Created by Shane Griffith on 8/5/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __SIFTFlow__AlignmentMachine__
#define __SIFTFlow__AlignmentMachine__

#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <vector>

class AlignmentMachine {
public:
    int machine_id=0;
    
    enum state{OPEN, LOCKED, RUNNING, FINISHED};
    
    void SetDebug(){
        debug=true;
    }
    virtual ~AlignmentMachine(){}
    int State() { return thread_state;}
    void SetSave(bool save){ _save = save; }
    bool CanSave(){return _save;};
    virtual void Reset() = 0;
    virtual void * Run() = 0;
    virtual void LogResults()=0;
protected:
    bool _save = true;
    bool debug = false;
    int thread_state = state::OPEN;
};



#endif /* defined(__SIFTFlow__AlignmentMachine__) */
