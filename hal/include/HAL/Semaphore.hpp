#pragma once

#include <stdint.h>
#include <condition_variable>
#include <pthread.h>
#include <semaphore.h>

// TODO: remote pthreads from the rest of HAL
typedef pthread_mutex_t* MUTEX_ID;
typedef sem_t* SEMAPHORE_ID;
typedef std::condition_variable::native_handle_type MULTIWAIT_ID;

extern "C"
{
	extern const uint32_t SEMAPHORE_Q_FIFO;
	extern const uint32_t SEMAPHORE_Q_PRIORITY;
	extern const uint32_t SEMAPHORE_DELETE_SAFE;
	extern const uint32_t SEMAPHORE_INVERSION_SAFE;

	extern const int32_t SEMAPHORE_NO_WAIT;
	extern const int32_t SEMAPHORE_WAIT_FOREVER;

	extern const uint32_t SEMAPHORE_EMPTY;
	extern const uint32_t SEMAPHORE_FULL;

	MUTEX_ID initializeMutexRecursive();
	MUTEX_ID initializeMutexNormal();
	void deleteMutex(MUTEX_ID sem);
	int8_t takeMutex(MUTEX_ID sem);
	int8_t tryTakeMutex(MUTEX_ID sem);
	int8_t giveMutex(MUTEX_ID sem);

	SEMAPHORE_ID initializeSemaphore(uint32_t initial_value);
	void deleteSemaphore(SEMAPHORE_ID sem);
	int8_t takeSemaphore(SEMAPHORE_ID sem);
	int8_t tryTakeSemaphore(SEMAPHORE_ID sem);
	int8_t giveSemaphore(SEMAPHORE_ID sem);

	MULTIWAIT_ID initializeMultiWait();
	void deleteMultiWait(MULTIWAIT_ID sem);
	int8_t takeMultiWait(MULTIWAIT_ID sem, MUTEX_ID m, int32_t timeout);
	int8_t giveMultiWait(MULTIWAIT_ID sem);
}

