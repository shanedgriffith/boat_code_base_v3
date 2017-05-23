//
//  MachineManager.cpp
//  SIFTFlow
//
//  Created by Shane Griffith on 8/19/15.
//  Copyright (c) 2015 shane. All rights reserved.
//

#include "MachineManager.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


void MachineManager::CleanupMachine(int j){
    if(debug)
        std::cout << "machine " << ams[j]->machine_id << " finished" << std::endl;
    
    if(ams[j]->State() != AlignmentMachine::FINISHED) return;
    
    if(ams[j]->CanSave()) ams[j]->LogResults();
    ams[j]->SetSave(true);
    ams[j]->Reset();
}

int MachineManager::OpenMachines(){
    int countopen = 0;
    for(int j=0; j<ams.size(); j++){
        if(ams[j]->State()==AlignmentMachine::FINISHED) CleanupMachine(j);
        if(ams[j]->State()==AlignmentMachine::OPEN) countopen++;
    }
    return countopen;
}

void MachineManager::WaitForMachine(bool bAll){
    while(1){
        int countopen = OpenMachines();
        if(countopen == ams.size()) break;
        else if(countopen > 0 && !bAll) break;
        usleep(5000);//5 ms
    }
}

bool MachineManager::WaitForAnotherMachine(){
    //As long as any machine is running, it waits for one to finish.
    //returns false if there are no more machines to wait for.
    int numfree=NumberOfFreeMachines();
    int countopen=numfree;
    while(countopen==numfree && countopen < ams.size()){
        usleep(5000);//5 ms
        countopen = OpenMachines();
    }
    if(countopen==ams.size()) return false;
    return true;
}

void MachineManager::TerminateMachines(){
    for(int j=0; j<ams.size(); j++){
        if(ams[j]->State() != AlignmentMachine::OPEN) ams[j]->SetSave(false);
    }
}

int MachineManager::NumberOfMachines(){
	return ams.size();
}

int MachineManager::NumberOfMachinesAtState(AlignmentMachine::state s){
    int count = 0;
    for(int i=0; i<ams.size(); i++)
        if(ams[i]->State()==s) count++;
    return count;
}

int MachineManager::NumberOfRunningMachines(){
    return NumberOfMachinesAtState(AlignmentMachine::LOCKED) + NumberOfMachinesAtState(AlignmentMachine::RUNNING);
}

int MachineManager::NumberOfFreeMachines(){
    return NumberOfMachinesAtState(AlignmentMachine::OPEN);
}

void MachineManager::States(){
    for(int i=0; i<ams.size(); i++)
        cout << "Machine " << i << " has state " << ams[i]->State() << endl;
}

bool MachineManager::CheckUsers(){
	static bool found_users = false;
	char line[2014];
	std::system("who >/tmp/check_users.txt"); // execute the UNIX command "ls -l >test.txt"
	FILE * fp = fopen("/tmp/check_users.txt", "r");
    if(!fp){
    	cout << "couldn't open: /tmp/check_users.txt" << endl;
    	exit(-1);
    }

	int count = 0;
	while(!feof(fp) && fgets(line, 1024, fp)){
		count++;
	}
	fclose(fp);

	if(count ==0 ){
		cout << "not setup correctly."<<endl;
	}
	else if(count > 1) {
		if(!found_users) cout << "Stopping process for user..."<<endl;
		found_users = true;
		return true;
	}
	if(found_users){
		cout << "Restarting process..."<<endl;
		found_users = false;
	}
	return false;
}

int MachineManager::GetOpenMachine(bool wait)
{
	if(check_users) while(CheckUsers()){
		if(NumberOfFreeMachines() != NumberOfMachines())
			WaitForMachine(true);
		else sleep(10);
	}
    if(wait) WaitForMachine(false);
    for(int j=0; j<ams.size(); j++){
        if(ams[j]->State() == AlignmentMachine::OPEN) return j;
    }
    return -1;
}

void * Thread_Alignment(void * arg)
{
    return static_cast<AlignmentMachine *>(arg)->Run();
}

void MachineManager::RunMachine(int j)
{
    pthread_t t;
    pthread_create(&t, 0, Thread_Alignment, ams[j]);
}


