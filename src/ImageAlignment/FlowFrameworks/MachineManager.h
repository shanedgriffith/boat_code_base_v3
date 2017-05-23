//
//  MachineManager.h
//  SIFTFlow
//
//  Created by Shane Griffith on 8/19/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef __SIFTFlow__MachineManager__
#define __SIFTFlow__MachineManager__

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "AlignmentMachine.h"

class MachineManager
{
private:
    int NumberOfMachinesAtState(AlignmentMachine::state s);
    int OpenMachines();
	bool CheckUsers();
public:
    bool check_users;
    bool debug = false;
    vector<AlignmentMachine*> ams;
    
    MachineManager(){
    	check_users = true;
    }

    void AddMachine(AlignmentMachine* am){
        am->machine_id = ams.size();
        ams.push_back(am);
    }
    void States();
    int NumberOfMachines();
    int NumberOfFreeMachines();
    int NumberOfRunningMachines();
    void WaitForMachine(bool bAll = false);
    bool WaitForAnotherMachine();
    int GetOpenMachine(bool wait = true);
    void RunMachine(int j);
    void CleanupMachine(int j);
    void TerminateMachines();
};



#endif /* defined(__SIFTFlow__MachineManager__) */
