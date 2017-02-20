/*
 * IRTask.h
 *
 *  Created on: Feb 20, 2017
 *      Author: Dylan Vos
 */

#ifndef IRTASK_H
#define IRTASK_H

#include <pthread.h>

namespace IR {

class IRTask {
public:
	IRTask(const char* taskName = NULL);
	virtual ~IRTask();

	static void* StarterTask(void* vtask);

	void Start();
	void Resume();
	void Pause();
	void Terminate();

	bool isEnabled();

	virtual void Run() = 0;

private:
	bool enabled_;
	bool running_;
	bool isDead_;
	pthread_t ptask;

};

} /* namespace IR */

#endif /* IRTASK_H */
