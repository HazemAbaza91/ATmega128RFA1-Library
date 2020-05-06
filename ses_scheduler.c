/*
 ***************************************************************************
 ses_scheduler V1 - Copyright (C) 2018 HAZEM ABAZA.
 ***************************************************************************
 This file is part of the SES_TUHH library.

 ses_scheduler is a library that allows scheduling different tasks.
 The scheduler has functions to allow its initialization , updating
 and running.It also has functions for adding and removing tasks.
 Each function of the scheduler (add , remove and update ) has its
 own pointer to the linked list to avoid any strange behaviors.

 ***************************************************************************
 */

/* INCLUDES ******************************************************************/
#include "ses_timer.h"
#include "ses_scheduler.h"
#include "util/atomic.h"
#include "ses_lcd.h"

/*-----------------------------------------------------------
 * Implementation of functions defined in scheduler.h *
 *----------------------------------------------------------*/

/* PRIVATE VARIABLES **************************************************/
/** list of scheduled tasks head */
static taskDescriptor* taskList = NULL;
volatile pTimerCallback myTimerCallback2 = NULL;
static systemTime_t time = 0;
/*FUNCTION DEFINITION *************************************************/

static void scheduler_update(void* m) {

	/* listLoopPointer is a pointer which points to
	 * all the task in the list and it starts from
	 * the taskList (List Head Node)
	 */

	taskDescriptor* listLoopPointer = taskList;

	while (listLoopPointer != NULL) {
		/* Check the expire time of the task.If the task has
		 * not been expired yet, the expire time for is
		 * decremented by one
		 */
		if ((listLoopPointer->expire) != 0) {
			listLoopPointer->expire--;
		}

		/* After checking the expire Time for the task, a task may expired.
		 * In this case , we have to raise the execute flag (execute =1) so
		 * the function will not wait for the next scheduler_update to update
		 * its execution flag.
		 */

		if (listLoopPointer->expire == 0) {
			/* Resetting the expire time to period
			 */

			listLoopPointer->expire = listLoopPointer->period;

			/* Raising the execution flag , indicating readiness for execution
			 */
			listLoopPointer->execute = 1;
		}

		/* listLoopPointer is set to point to the next  task
		 */

		listLoopPointer = listLoopPointer->next;
	}

}

void scheduler_init() {

	/*
	 * The scheduler update function passed to
	 * set timer 2 call back
	 */
	pTimerCallback pSchedulerUpdate = &scheduler_update;

	timer2_setCallback(pSchedulerUpdate);

	/*Timer 2 is started*/

	timer2_start();

}

void scheduler_run() {

	/*A pointer is created to point to the current task
	 * when running the scheduler. It is initialized
	 * to point on the head of the list at the begining.
	 */

	taskDescriptor* currentNode = taskList;

	while (1) {

		/*
		 * The scheduler_run loops on the task list
		 * till the last node (the Empty Node)
		 */

		if (currentNode != NULL) {

			/*
			 * The scheduler_run checks the execute flag
			 * status of every task. IF it is true , It
			 * continue to check whether the task is periodic
			 * or non-periodic
			 */

			if (currentNode->execute) {

				/* If the task is periodic,the task is executed
				 * ,the execute flag is turned off and then the
				 * current node is updated
				 */

				if (currentNode->period > 0) {

					currentNode->task(currentNode->param);

					currentNode->execute = 0;

					currentNode = currentNode->next;

				}

				/* If the task is non-periodic,the task is executed
				 * ,the execute flag is turned off and then the
				 * current node is updated.
				 */

				else if (currentNode->period == 0) {

					currentNode->task(currentNode->param);

					scheduler_remove(currentNode);
					currentNode = currentNode->next;

				}

			}

			/* If execute flag is not raised (False),
			 * the cuurentNode pointer should increment
			 * to the next task.
			 */

			else {
				currentNode = currentNode->next;
			}
		}

		/* If the node at the task list is empty(i.e Last Node),
		 * the current Node pointer is set to the task head
		 */

		else {
			currentNode = taskList;

		}
	}
}

bool scheduler_add(taskDescriptor * toAdd) {

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		taskDescriptor* currentNode = taskList;
		taskDescriptor* previousNode = taskList;

		/* Check that the passed task to the scheduler
		 * is not NULL.If the task is NULL, the add function
		 * should terminate and return false.
		 */

		if (toAdd == NULL) {
			return false;
		}

		/* If the passed task to the scheduler_add function is not NULL,
		 * We should decide where should it placed inside the linked List
		 * There are two cases for adding this toAdd task
		 */

		/*-------------------------------------------------------------------------
		 * CASE 1  : Adding a Node to the begining of the task
		 *-------------------------------------------------------------------------*/
		/* The toAdd task is set to be taskLits (i.e The Task Head)
		 and taskList->next is set to NULL.

		 head          second         third
		 |             |              |
		 |             |              |
		 +---+---+     +----+----+    +-----+----+
		 | 1  | o--->  | #  | #  |    |  #  | #  |
		 +---+---+     +----+----+    +-----+----+

		 */

		if (taskList == NULL) {
			/*
			 * toAdd becomes the head of the task list.
			 */

			taskList = toAdd;
			/*
			 * taskList->next now is pointing to NULL as it
			 * is the first element to be added to the list.
			 */
			taskList->next = NULL;
			/*
			 * taskList->next now is pointing to NULL as it
			 * is the first element to be added to the list.
			 */

		}

		/*--------------------------------------------------------------------
		 * CASE 2 : Adding Node to the end of the list
		 *---------------------------------------------------------------------*/
		/* We loop over the list of tasks to find the last Node (First Empty Node).
		 Once it is found, we add our task to it and create a new node at the end.
		 The drawings below is just for demonstration.


		 head         second         third
		 |             |             |
		 |             |             |
		 +---+---+     +---+---+     +----+----+
		 | 1  | o----->| 2 | o-----> |  # |  # |
		 +---+---+     +---+---+     +----+----+
		 */

		else {

			while (currentNode != NULL) {
				/* Important check has to be made here.If we are adding
				 * an already existing task.The scheduler_add function
				 * should terminate and return false.
				 */

				if (toAdd == currentNode) {
					return false;
				}

				/* When Looping , the previousNode and currentNode
				 *pointers are updated to the current and previous
				 *nodes in the list.
				 */
				previousNode = currentNode;

				currentNode = currentNode->next;

			}

			/* Once the last node is found , The
			 * new toAdd task is added atomically
			 */

			previousNode->next = toAdd;

		}

		/* After Adding the toAddd , the previousNode and currentNode
		 *pointers are updated to the current and previous
		 *nodes in the list.
		 */
		currentNode = taskList;

		previousNode = taskList;

		/*
		 * Once the task is added , the function returns true
		 * indicating process success.
		 */
	}
	return true;

}

/*void scheduler_remove(taskDescriptor* toRemove) {

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		taskDescriptor* currentNode = taskList;
		taskDescriptor* previousNode = taskList;

		 When passing a task to the scheduler_remove function, Its place in
		 * task list should be determined so that an approperiate remove task is
		 * performed depending on its position in the list
		 * There are three cases for adding this toAdd task


		-------------------------------------------------------------------------
		 * CASE 1  : Removing The First Node
		 *-------------------------------------------------------------------------
		 We loop over the list of tasks to find the last Node (First Empty Node).
		 Once it is found, we add our task to it and create a new node at the end.
		 The drawings below is just for demonstration.


		 head            Next Node
		 |               |
		 |               |
		 +---+---+  ||   +---+---+
		 | 1  | o---||-->| # | #----->
		 +---+---+  ||   +---+---+

		If the required task to remove is the task list head, Temporary node
		 * is created called (nextNode).taskList--points-->nextNode,
		 * Then the task head is removed from the task and the temperoray
		 * node(next node) becomes the list head.


		if (taskList == toRemove) {

			taskDescriptor* nextNode = taskList->next;

			taskList = NULL;
			taskList = nextNode;
			currentNode = taskList;
			previousNode = taskList;

			return;

		}

		-------------------------------------------------------------------------
		 * CASE 2  : Removing The Last Node
		 *-------------------------------------------------------------------------

		If the required task to remove is the last task, The
		 * The previousNode->next is set to NULL and then
		 * the current node is freed.

		 Previous Node               Last Node
		 |                          |
		 |                          |
		 +---+---+                  +----+----+
		 | 2 | o-----> (NULL)       |  # |  # |
		 +---+---+                  +----+----+



		else if (currentNode->next == NULL)

		{

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				previousNode->next = NULL;
				currentNode = NULL;
			}
			return;

		}

		-------------------------------------------------------------------------
		 * CASE 3 : Removing Intermediate Node from the Task List
		 *-------------------------------------------------------------------------
		 We loop over the list of tasks to find the required task to remove.
		 Once it is found, We set our previous node to pint to the next one, then.
		 we freed the intermediate current one.


		 head          second         third
		 |              |             |
		 |              |             |
		 +---+---+      +---+---+     +----+----+
		 | 1  | o-----> | 2 | o--||-  | 3  |  0 |
		 +---+---+  |   +---+---+     +----+----+
		 |                  ^
		 |------>>>---------|



		else {
			while (currentNode->next != NULL) {

				if (currentNode == toRemove) {

					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						previousNode->next = currentNode->next;
						currentNode = NULL;
						currentNode = previousNode->next;
					}

					return;
				}

				else {
					 previous and current nodes are
					 * updated till the toRemove is found

					previousNode = currentNode;
					currentNode = currentNode->next;
				}
			}

		}
	}
}*/


void scheduler_remove(taskDescriptor * toRemove) {

	taskDescriptor* temp = taskList;
	taskDescriptor* pointer;

	/* if task list has only one task and the task should be removed */

	if ((*temp).next == NULL && (temp == toRemove))
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			taskList = NULL;
		}

	/* if task list has more than one task and the first task should be removed */

	else if (temp == toRemove) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			pointer = (*temp).next;
			(*temp).next = NULL;
			temp = pointer;
			taskList = temp;
		}

		/* if task list has more than one task and the task that should be removed is not the first task in the list */

	} else {
		while ((*temp).next != toRemove && (*temp).next != NULL) {
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				temp = (*temp).next;
			}
		}
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			pointer = (*((*temp).next)).next;
			(*((*temp).next)).next = NULL;
			(*temp).next = pointer;

		}
	}
}

systemTime_t scheduler_getTime() {
	return time;
}

void scheduler_setTime(systemTime_t a) {
	time=a;
}


void timer2_setCallback(pTimerCallback cb) {

	/*
	 * myTimerCallback pointer is set by the passed
	 * function from the user
	 */
	myTimerCallback2 = cb;
}

ISR(TIMER2_COMPA_vect) {

	time++;

	if(time>86399000)
	{
		time=0;
	}

	if (myTimerCallback2 != NULL) {
		myTimerCallback2(NULL);
	}
}

