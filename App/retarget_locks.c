// -------------------- Retarget Locks --------------------
//
// share/doc/gcc-arm-none-eabi/pdf/libc.pdf:
//
// Newlib was configured to allow the target platform to provide the locking routines and
// static locks at link time. As such, a dummy default implementation of these routines and
// static locks is provided for single-threaded application to link successfully out of the box on
// bare-metal systems.
//
// For multi-threaded applications the target platform is required to provide an implementa-
// tion for *all* these routines and static locks. If some routines or static locks are missing, the
// link will fail with doubly defined symbols.
//
// (see also newlib/libc/misc/lock.c)
//

#include "retarget_locks.h"
#include <sys/lock.h>
#include <newlib.h>
#include <stdatomic.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


struct __lock {
	SemaphoreHandle_t sem;
};

struct __lock __lock___sharedRecursive __attribute__((section(".shared")));
struct __lock __lock___shared __attribute__((section(".shared")));
struct __lock __lock___streams __attribute__((section(".shared")));
struct __lock __lock___streamsRecursive __attribute__((section(".shared")));
struct __lock __lock___cxa_guard __attribute__((section(".shared")));
struct __lock __lock___malloc __attribute__((section(".shared")));

struct __lock __lock___sinit_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___sfp_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___atexit_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___at_quick_exit_mutex __attribute__((section(".shared")));
struct __lock __lock___malloc_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___env_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___tz_mutex __attribute__((section(".shared")));
struct __lock __lock___dd_hash_mutex __attribute__((section(".shared")));
struct __lock __lock___arc4random_mutex __attribute__((section(".shared")));

void retarget_locks_init(void) {
	__lock___sharedRecursive.sem = xSemaphoreCreateRecursiveMutex();
	__lock___shared.sem = xSemaphoreCreateMutex();
	__lock___streams.sem = xSemaphoreCreateMutex();
	__lock___streamsRecursive.sem = xSemaphoreCreateRecursiveMutex();
	__lock___cxa_guard.sem = xSemaphoreCreateMutex();
	__lock___malloc.sem = xSemaphoreCreateRecursiveMutex();

	__lock___sinit_recursive_mutex = __lock___sharedRecursive;
	__lock___sfp_recursive_mutex = __lock___sharedRecursive;
	__lock___atexit_recursive_mutex = __lock___sharedRecursive;
	__lock___at_quick_exit_mutex = __lock___shared;
	__lock___malloc_recursive_mutex = __lock___malloc;
	__lock___env_recursive_mutex = __lock___sharedRecursive;
	__lock___tz_mutex = __lock___shared;
	__lock___dd_hash_mutex = __lock___shared;
	__lock___arc4random_mutex = __lock___shared;
}

void retarget_locks_grant_access(TaskHandle_t task) {
	vGrantAccessToSemaphore(task, __lock___sharedRecursive.sem);
	vGrantAccessToSemaphore(task, __lock___shared.sem);
	vGrantAccessToSemaphore(task, __lock___streams.sem);
	vGrantAccessToSemaphore(task, __lock___streamsRecursive.sem);
	vGrantAccessToSemaphore(task, __lock___cxa_guard.sem);
	vGrantAccessToSemaphore(task, __lock___malloc.sem);
}


void __retarget_lock_init(_LOCK_T *lock_ptr) {
	(*lock_ptr) = &__lock___streams;
}


void __retarget_lock_init_recursive(_LOCK_T *lock_ptr) {
	(*lock_ptr) = &__lock___streamsRecursive;
}


void __retarget_lock_close(_LOCK_T lock) {
}


void __retarget_lock_close_recursive(_LOCK_T lock) {
}


void __retarget_lock_acquire(_LOCK_T lock) {
	if (xSemaphoreTake(lock->sem, portMAX_DELAY) != pdTRUE) {
		Error_Handler();
	}
}


void __retarget_lock_acquire_recursive(_LOCK_T lock) {
	if (xSemaphoreTakeRecursive(lock->sem, portMAX_DELAY) != pdTRUE) {
		Error_Handler();
	}
}


int __retarget_lock_try_acquire(_LOCK_T lock) {
    return xSemaphoreTake(lock->sem, 0);
}


int __retarget_lock_try_acquire_recursive(_LOCK_T lock) {
    return xSemaphoreTakeRecursive(lock->sem, 0);
}


void __retarget_lock_release(_LOCK_T lock) {
    if (xSemaphoreGive(lock->sem) != pdTRUE) {
		Error_Handler();
	}
}


void __retarget_lock_release_recursive(_LOCK_T lock) {
    if (xSemaphoreGiveRecursive(lock->sem) != pdTRUE) {
		Error_Handler();
	}
}


/**
  * @defgroup __cxa_guard_ GNU C++ one-time construction API
  * @see https://itanium-cxx-abi.github.io/cxx-abi/abi.html#once-ctor
  *
  * When building for C++, please make sure that <tt>-fno-threadsafe-statics</tt> is not passed to the compiler
  * @{
  */

/** The guard object is created by the C++ compiler and is 32 bit for ARM EABI. */
typedef struct
{
	atomic_uchar initialized; /**< Indicate if object is initialized */
	uint8_t acquired; /**< Ensure non-recursive lock */
	uint16_t unused; /**< Padding */
} __attribute__((packed)) CxaGuardObject_t;


/**
  * @brief Acquire __cxa_guard mutex
  * @param guard_object Guard object
  * @return 0 if object is initialized, else initialization of object required
  */
int __cxa_guard_acquire(CxaGuardObject_t *guard_object) {
	if (atomic_load(&guard_object->initialized) == 0) {
		/* Object needs initialization, lock threading context */
		__retarget_lock_acquire(&__lock___cxa_guard);
		if (atomic_load(&guard_object->initialized) == 0) {
			/* Object needs initialization */
			if (guard_object->acquired) {
				/* Object initialization already in progress */
				Error_Handler();
			}

			/* Lock acquired */
			guard_object->acquired = 1;
			return 1;
		} else {
			/* Object initialized in another thread */
			__retarget_lock_release(&__lock___cxa_guard);
		}
	}

  /* Object already initialized */
  return 0;
}

/**
  * @brief Abort __cxa_guard mutex
  * @param guard_object Guard object
  */
void __cxa_guard_abort(CxaGuardObject_t *guard_object) {
	if (guard_object->acquired) {
		/* Release lock */
		guard_object->acquired = 0;
		__retarget_lock_release(&__lock___cxa_guard);
	} else {
		/* Trying to release non-acquired lock */
		Error_Handler();
	}
}

/**
  * @brief Release __cxa_guard mutex
  * @param guard_object Guard object
  */
void __cxa_guard_release(CxaGuardObject_t *guard_object) {
	/* Object initialized */
	atomic_store(&guard_object->initialized, 1);

	/* Release lock */
	__cxa_guard_abort(guard_object);
}
