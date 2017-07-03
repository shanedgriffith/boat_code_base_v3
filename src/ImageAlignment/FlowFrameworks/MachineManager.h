//
//  MachineManager.h
//  SIFTFlow
//
//  Created by Shane Griffith on 8/19/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#ifndef SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_MACHINEMANAGER_H_
#define SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_MACHINEMANAGER_H_

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "AlignmentMachine.h"

class MachineManager {
private:
    int NumberOfMachinesAtState(AlignmentMachine::state s);
    int OpenMachines();
	bool CheckUsers();
public:
    bool check_users;
    bool debug = false;
    std::vector<AlignmentMachine*> ams;
    
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



#endif /* SRC_IMAGEALIGNMENT_FLOWFRAMEWORKS_MACHINEMANAGER_H_ */
