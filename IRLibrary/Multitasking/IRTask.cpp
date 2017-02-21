/*
 * IRTask.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: Dylan Vos
 */

#include "IRTask.h"
#include <string.h>
#include <Utility.h>
#include <Timer.h>

namespace IR {

IRTask::IRTask(const char* taskName) {
	char old_name[128];

	if(!taskName){
		sprintf(old_name, "IRTask-%llu", frc::GetFPGATime());
	}

	enabled_ = false;
	running_ = true;
	isDead_ = false;

	pthread_create(&ptask, NULL, &IRTask::StarterTask, (void*)this);
}

IRTask::~IRTask(){
	void* res;
	this->Terminate();
	pthread_join(ptask, &res);
	free(res);
}

void* IRTask::StarterTask(void* vtask){
	IRTask* task = (IRTask*)vtask;

	while(task->running_){
		if(task->enabled_){
			task->Run();
			frc::Wait(0.002); //Wait for active
		} else {
			frc::Wait(0.05); //Wait while paused
		}
	}

	task->isDead_ = true;
	return NULL;
}

void IRTask::Start(){
	enabled_ = true;
}

void IRTask::Resume(){
	enabled_ = false;
}

void IRTask::Pause(){
	enabled_ = false;
}

void IRTask::Terminate(){
	running_ = false;

	while(!isDead_){
		frc::Wait(0.02);
	}
}

bool IRTask::isEnabled(){
	return enabled_;
}

} /* namespace IR */
