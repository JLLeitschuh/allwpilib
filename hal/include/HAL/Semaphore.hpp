#pragma once

#include <stdint.h>
#include <pthread.h>
#include <semaphore.h>
#include <mutex>
#include <condition_variable>

typedef std::mutex* MUTEX_ID;
typedef std::condition_variable* MULTIWAIT_ID;
typedef pthread_cond_t* NATIVE_MULTIWAIT_ID;

extern "C"
{
	MUTEX_ID initializeMutexNormal();
	void deleteMutex(MUTEX_ID sem);
	void takeMutex(MUTEX_ID sem);
	bool tryTakeMutex(MUTEX_ID sem);
	void giveMutex(MUTEX_ID sem);

	MULTIWAIT_ID initializeMultiWait();
	void deleteMultiWait(MULTIWAIT_ID sem);
	void takeMultiWait(MULTIWAIT_ID sem, MUTEX_ID m);
	void giveMultiWait(MULTIWAIT_ID sem);
}

