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
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


struct __lock {
    SemaphoreHandle_t   sem;
};

struct __lock __lock___sinit_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___sfp_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___atexit_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___at_quick_exit_mutex __attribute__((section(".shared")));
struct __lock __lock___malloc_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___env_recursive_mutex __attribute__((section(".shared")));
struct __lock __lock___tz_mutex __attribute__((section(".shared")));
struct __lock __lock___dd_hash_mutex __attribute__((section(".shared")));
struct __lock __lock___arc4random_mutex __attribute__((section(".shared")));

void retarget_locks_init(void)
{
    __lock___sinit_recursive_mutex.sem  = xSemaphoreCreateRecursiveMutex();
    __lock___sfp_recursive_mutex.sem    = xSemaphoreCreateRecursiveMutex();
    __lock___atexit_recursive_mutex.sem = xSemaphoreCreateRecursiveMutex();
    __lock___at_quick_exit_mutex.sem    = xSemaphoreCreateMutex();
    __lock___malloc_recursive_mutex.sem = xSemaphoreCreateRecursiveMutex();
    __lock___env_recursive_mutex.sem    = xSemaphoreCreateRecursiveMutex();
    __lock___tz_mutex.sem               = xSemaphoreCreateMutex();
    __lock___dd_hash_mutex.sem          = xSemaphoreCreateMutex();
    __lock___arc4random_mutex.sem       = xSemaphoreCreateMutex();
}


void __retarget_lock_init(_LOCK_T *lock_ptr)
{
//    *lock_ptr = pvPortMalloc(sizeof(struct __lock));
//    (*lock_ptr)->sem = xSemaphoreCreateMutex();
}


void __retarget_lock_init_recursive(_LOCK_T *lock_ptr)
{
//    *lock_ptr = pvPortMalloc(sizeof(struct __lock));
//    (*lock_ptr)->sem = xSemaphoreCreateRecursiveMutex();
}


void __retarget_lock_close(_LOCK_T lock)
{
//    vSemaphoreDelete(lock->sem);
//    vPortFree(lock);
}


void __retarget_lock_close_recursive(_LOCK_T lock)
{
//    vSemaphoreDelete(lock->sem);
//    vPortFree(lock);
}


void __retarget_lock_acquire(_LOCK_T lock)
{
    xSemaphoreTake(lock->sem, portMAX_DELAY);
}


void __retarget_lock_acquire_recursive(_LOCK_T lock)
{
    xSemaphoreTakeRecursive(lock->sem, portMAX_DELAY);
}


int __retarget_lock_try_acquire(_LOCK_T lock)
{
    return xSemaphoreTake(lock->sem, 0);
}


int __retarget_lock_try_acquire_recursive(_LOCK_T lock)
{
    return xSemaphoreTakeRecursive(lock->sem, 0);
}


void __retarget_lock_release(_LOCK_T lock)
{
    xSemaphoreGive(lock->sem);
}


void __retarget_lock_release_recursive(_LOCK_T lock)
{
    xSemaphoreGiveRecursive(lock->sem);
}
