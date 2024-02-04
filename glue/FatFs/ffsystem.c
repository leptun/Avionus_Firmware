/*------------------------------------------------------------------------*/
/* A Sample Code of User Provided OS Dependent Functions for FatFs        */
/*------------------------------------------------------------------------*/

#include "ff.h"
#include "main.h"


#if FF_FS_REENTRANT	/* Mutal exclusion */
/*------------------------------------------------------------------------*/
/* Definitions of Mutex                                                   */
/*------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "semphr.h"
static SemaphoreHandle_t ff_Mutex[FF_VOLUMES + 1];	/* Table of mutex handle */

void ff_system_init() {
	for (int vol = 0; vol < FF_VOLUMES + 1; vol++) {
		ff_Mutex[vol] = xSemaphoreCreateMutex();
		if (!ff_Mutex[vol]) {
			Error_Handler();
		}
	}
}

void ff_system_grant_access(TaskHandle_t task) {
	for (int vol = 0; vol < FF_VOLUMES + 1; vol++) {
		vGrantAccessToEventGroup(task, ff_Mutex[vol]);
	}
}


/*------------------------------------------------------------------------*/
/* Create a Mutex                                                         */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount function to create a new mutex
/  or semaphore for the volume. When a 0 is returned, the f_mount function
/  fails with FR_INT_ERR.
*/

int ff_mutex_create (	/* Returns 1:Function succeeded or 0:Could not create the mutex */
	int vol				/* Mutex ID: Volume mutex (0 to FF_VOLUMES - 1) or system mutex (FF_VOLUMES) */
)
{
//	ff_Mutex[vol] = xSemaphoreCreateMutex();
	return (int)(ff_Mutex[vol] != NULL);
}


/*------------------------------------------------------------------------*/
/* Delete a Mutex                                                         */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount function to delete a mutex or
/  semaphore of the volume created with ff_mutex_create function.
*/

void ff_mutex_delete (	/* Returns 1:Function succeeded or 0:Could not delete due to an error */
	int vol				/* Mutex ID: Volume mutex (0 to FF_VOLUMES - 1) or system mutex (FF_VOLUMES) */
)
{
//	vSemaphoreDelete(ff_Mutex[vol]);
}


/*------------------------------------------------------------------------*/
/* Request a Grant to Access the Volume                                   */
/*------------------------------------------------------------------------*/
/* This function is called on enter file functions to lock the volume.
/  When a 0 is returned, the file function fails with FR_TIMEOUT.
*/

int ff_mutex_take (	/* Returns 1:Succeeded or 0:Timeout */
	int vol			/* Mutex ID: Volume mutex (0 to FF_VOLUMES - 1) or system mutex (FF_VOLUMES) */
)
{
	return (int)(xSemaphoreTake(ff_Mutex[vol], FF_FS_TIMEOUT) == pdTRUE);
}



/*------------------------------------------------------------------------*/
/* Release a Grant to Access the Volume                                   */
/*------------------------------------------------------------------------*/
/* This function is called on leave file functions to unlock the volume.
*/

void ff_mutex_give (
	int vol			/* Mutex ID: Volume mutex (0 to FF_VOLUMES - 1) or system mutex (FF_VOLUMES) */
)
{
	xSemaphoreGive(ff_Mutex[vol]);
}

#endif	/* FF_FS_REENTRANT */

