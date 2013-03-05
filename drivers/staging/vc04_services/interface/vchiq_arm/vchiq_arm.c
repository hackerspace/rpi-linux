/**
 * Copyright (c) 2010-2012 Broadcom. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/bug.h>
#include <linux/semaphore.h>
#include <linux/list.h>

#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "vchiq_core.h"
#include "vchiq_arm.h"

#define DEVICE_NAME "vchiq"

/* Override the default prefix, which would be vchiq_arm (from the filename) */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX DEVICE_NAME "."

#define VCHIQ_MINOR 0

/* Some per-instance constants */
#define MAX_COMPLETIONS 16
#define MAX_SERVICES 64
#define MAX_ELEMENTS 8
#define MSG_QUEUE_SIZE 64

#define KEEPALIVE_VER 1
#define KEEPALIVE_VER_MIN KEEPALIVE_VER

/* Run time control of log level, based on KERN_XXX level. */
int vchiq_arm_log_level = VCHIQ_LOG_DEFAULT;
int vchiq_susp_log_level = VCHIQ_LOG_ERROR;

#define SUSPEND_TIMER_TIMEOUT_MS 100
#define SUSPEND_RETRY_TIMER_TIMEOUT_MS 1000

#define VC_SUSPEND_NUM_OFFSET 3 /* number of values before idle which are -ve */
static const char *const suspend_state_names[] = {
	"VC_SUSPEND_FORCE_CANCELED",
	"VC_SUSPEND_REJECTED",
	"VC_SUSPEND_FAILED",
	"VC_SUSPEND_IDLE",
	"VC_SUSPEND_REQUESTED",
	"VC_SUSPEND_IN_PROGRESS",
	"VC_SUSPEND_SUSPENDED"
};
#define VC_RESUME_NUM_OFFSET 1 /* number of values before idle which are -ve */
static const char *const resume_state_names[] = {
	"VC_RESUME_FAILED",
	"VC_RESUME_IDLE",
	"VC_RESUME_REQUESTED",
	"VC_RESUME_IN_PROGRESS",
	"VC_RESUME_RESUMED"
};
/* The number of times we allow force suspend to timeout before actually
** _forcing_ suspend.  This is to cater for SW which fails to release vchiq
** correctly - we don't want to prevent ARM suspend indefinitely in this case.
*/
#define FORCE_SUSPEND_FAIL_MAX 8

/* The time in ms allowed for videocore to go idle when force suspend has been
 * requested */
#define FORCE_SUSPEND_TIMEOUT_MS 200


static void suspend_timer_callback(unsigned long context);


typedef struct user_service_struct {
	VCHIQ_SERVICE_T *service;
	void *userdata;
	VCHIQ_INSTANCE_T instance;
	int is_vchi;
	int dequeue_pending;
	int message_available_pos;
	int msg_insert;
	int msg_remove;
	struct semaphore insert_event;
	struct semaphore remove_event;
	VCHIQ_HEADER_T * msg_queue[MSG_QUEUE_SIZE];
} USER_SERVICE_T;

struct bulk_waiter_node {
	struct bulk_waiter bulk_waiter;
	int pid;
	struct list_head list;
};

struct vchiq_instance_struct {
	VCHIQ_STATE_T *state;
	int completion_insert;
	int completion_remove;
	struct semaphore insert_event;
	struct semaphore remove_event;
	struct mutex completion_mutex;

	int connected;
	int closing;
	int pid;
	int mark;

	struct list_head bulk_waiter_list;
	struct mutex bulk_waiter_list_mutex;
};

typedef struct dump_context_struct {
	char __user *buf;
	size_t actual;
	size_t space;
	loff_t offset;
} DUMP_CONTEXT_T;

static VCHIQ_STATE_T g_state;
static DEFINE_SPINLOCK(msg_queue_spinlock);

/****************************************************************************
*
*   add_completion
*
***************************************************************************/

static VCHIQ_STATUS_T
add_completion(VCHIQ_INSTANCE_T instance, VCHIQ_REASON_T reason,
	VCHIQ_HEADER_T *header, USER_SERVICE_T *user_service,
	void *bulk_userdata)
{
	DEBUG_INITIALISE(g_state.local)

	while (instance->completion_insert ==
		(instance->completion_remove + MAX_COMPLETIONS)) {
		/* Out of space - wait for the client */
		DEBUG_TRACE(SERVICE_CALLBACK_LINE);
		vchiq_log_trace(vchiq_arm_log_level,
			"add_completion - completion queue full");
		DEBUG_COUNT(COMPLETION_QUEUE_FULL_COUNT);
		if (down_interruptible(&instance->remove_event) != 0) {
			vchiq_log_info(vchiq_arm_log_level,
				"service_callback interrupted");
			return VCHIQ_RETRY;
		} else if (instance->closing) {
			vchiq_log_info(vchiq_arm_log_level,
				"service_callback closing");
			return VCHIQ_ERROR;
		}
		DEBUG_TRACE(SERVICE_CALLBACK_LINE);
	}

	if (reason == VCHIQ_SERVICE_CLOSED)
		/* Take an extra reference, to be held until
		   this CLOSED notification is delivered. */
		lock_service(user_service->service);

	/* A write barrier is needed here to ensure that the entire completion
		record is written out before the insert point. */
	wmb();

	if (reason == VCHIQ_MESSAGE_AVAILABLE)
		user_service->message_available_pos =
			instance->completion_insert;
	instance->completion_insert++;

	up(&instance->insert_event);

	return VCHIQ_SUCCESS;
}

/****************************************************************************
*
*   service_callback
*
***************************************************************************/

static VCHIQ_STATUS_T
service_callback(VCHIQ_REASON_T reason, VCHIQ_HEADER_T *header,
	VCHIQ_SERVICE_HANDLE_T handle, void *bulk_userdata)
{
	/* How do we ensure the callback goes to the right client?
	** The service_user data points to a USER_SERVICE_T record containing
	** the original callback and the user state structure, which contains a
	** circular buffer for completion records.
	*/
	USER_SERVICE_T *user_service;
	VCHIQ_SERVICE_T *service;
	VCHIQ_INSTANCE_T instance;
	DEBUG_INITIALISE(g_state.local)

	DEBUG_TRACE(SERVICE_CALLBACK_LINE);

	service = handle_to_service(handle);
	BUG_ON(!service);
	user_service = (USER_SERVICE_T *)service->base.userdata;
	instance = user_service->instance;

	if (!instance || instance->closing)
		return VCHIQ_SUCCESS;

	vchiq_log_trace(vchiq_arm_log_level,
		"service_callback - service %lx(%d), reason %d, header %lx, "
		"instance %lx, bulk_userdata %lx",
		(unsigned long)user_service,
		service->localport,
		reason, (unsigned long)header,
		(unsigned long)instance, (unsigned long)bulk_userdata);

	if (header && user_service->is_vchi) {
		spin_lock(&msg_queue_spinlock);
		while (user_service->msg_insert ==
			(user_service->msg_remove + MSG_QUEUE_SIZE)) {
			spin_unlock(&msg_queue_spinlock);
			DEBUG_TRACE(SERVICE_CALLBACK_LINE);
			DEBUG_COUNT(MSG_QUEUE_FULL_COUNT);
			vchiq_log_trace(vchiq_arm_log_level,
				"service_callback - msg queue full");
			/* If there is no MESSAGE_AVAILABLE in the completion
			** queue, add one
			*/
			if ((user_service->message_available_pos -
				instance->completion_remove) < 0) {
				VCHIQ_STATUS_T status;
				vchiq_log_info(vchiq_arm_log_level,
					"Inserting extra MESSAGE_AVAILABLE");
				DEBUG_TRACE(SERVICE_CALLBACK_LINE);
				status = add_completion(instance, reason,
					NULL, user_service, bulk_userdata);
				if (status != VCHIQ_SUCCESS) {
					DEBUG_TRACE(SERVICE_CALLBACK_LINE);
					return status;
				}
			}

			DEBUG_TRACE(SERVICE_CALLBACK_LINE);
			if (down_interruptible(&user_service->remove_event)
				!= 0) {
				vchiq_log_info(vchiq_arm_log_level,
					"service_callback interrupted");
				DEBUG_TRACE(SERVICE_CALLBACK_LINE);
				return VCHIQ_RETRY;
			} else if (instance->closing) {
				vchiq_log_info(vchiq_arm_log_level,
					"service_callback closing");
				DEBUG_TRACE(SERVICE_CALLBACK_LINE);
				return VCHIQ_ERROR;
			}
			DEBUG_TRACE(SERVICE_CALLBACK_LINE);
			spin_lock(&msg_queue_spinlock);
		}

		user_service->msg_queue[user_service->msg_insert &
			(MSG_QUEUE_SIZE - 1)] = header;
		user_service->msg_insert++;
		spin_unlock(&msg_queue_spinlock);

		up(&user_service->insert_event);

		/* If there is a thread waiting in DEQUEUE_MESSAGE, or if
		** there is a MESSAGE_AVAILABLE in the completion queue then
		** bypass the completion queue.
		*/
		if (((user_service->message_available_pos -
			instance->completion_remove) >= 0) ||
			user_service->dequeue_pending) {
			DEBUG_TRACE(SERVICE_CALLBACK_LINE);
			user_service->dequeue_pending = 0;
			return VCHIQ_SUCCESS;
		}

		header = NULL;
	}
	DEBUG_TRACE(SERVICE_CALLBACK_LINE);

	return add_completion(instance, reason, header, user_service,
		bulk_userdata);
}

/****************************************************************************
*
*   vchiq_dump
*
***************************************************************************/

void
vchiq_dump(void *dump_context, const char *str, int len)
{
	DUMP_CONTEXT_T *context = (DUMP_CONTEXT_T *)dump_context;

	if (context->actual < context->space) {
		int copy_bytes;
		if (context->offset > 0) {
			int skip_bytes = min(len, (int)context->offset);
			str += skip_bytes;
			len -= skip_bytes;
			context->offset -= skip_bytes;
			if (context->offset > 0)
				return;
		}
		copy_bytes = min(len, (int)(context->space - context->actual));
		if (copy_bytes == 0)
			return;
		if (copy_to_user(context->buf + context->actual, str,
			copy_bytes))
			context->actual = -EFAULT;
		context->actual += copy_bytes;
		len -= copy_bytes;

		/* If tne terminating NUL is included in the length, then it
		** marks the end of a line and should be replaced with a
		** carriage return. */
		if ((len == 0) && (str[copy_bytes - 1] == '\0')) {
			char cr = '\n';
			if (copy_to_user(context->buf + context->actual - 1,
				&cr, 1))
				context->actual = -EFAULT;
		}
	}
}

/****************************************************************************
*
*   vchiq_dump_platform_instance_state
*
***************************************************************************/

void
vchiq_dump_platform_instances(void *dump_context)
{
	VCHIQ_STATE_T *state = vchiq_get_state();
	char buf[80];
	int len;
	int i;

	/* There is no list of instances, so instead scan all services,
		marking those that have been dumped. */

	for (i = 0; i < state->unused_service; i++) {
		VCHIQ_SERVICE_T *service = state->services[i];
		VCHIQ_INSTANCE_T instance;

		if (service && (service->base.callback == service_callback)) {
			instance = service->instance;
			if (instance)
				instance->mark = 0;
		}
	}

	for (i = 0; i < state->unused_service; i++) {
		VCHIQ_SERVICE_T *service = state->services[i];
		VCHIQ_INSTANCE_T instance;

		if (service && (service->base.callback == service_callback)) {
			instance = service->instance;
			if (instance && !instance->mark) {
				len = snprintf(buf, sizeof(buf),
					"Instance %x: pid %d,%s completions "
						"%d/%d",
					(unsigned int)instance, instance->pid,
					instance->connected ? " connected, " :
						"",
					instance->completion_insert -
						instance->completion_remove,
					MAX_COMPLETIONS);

				vchiq_dump(dump_context, buf, len + 1);

				instance->mark = 1;
			}
		}
	}
}

/****************************************************************************
*
*   vchiq_dump_platform_service_state
*
***************************************************************************/

void
vchiq_dump_platform_service_state(void *dump_context, VCHIQ_SERVICE_T *service)
{
	USER_SERVICE_T *user_service = (USER_SERVICE_T *)service->base.userdata;
	char buf[80];
	int len;

	len = snprintf(buf, sizeof(buf), "  instance %x",
		(unsigned int)service->instance);

	if ((service->base.callback == service_callback) &&
		user_service->is_vchi) {
		len += snprintf(buf + len, sizeof(buf) - len,
			", %d/%d messages",
			user_service->msg_insert - user_service->msg_remove,
			MSG_QUEUE_SIZE);

		if (user_service->dequeue_pending)
			len += snprintf(buf + len, sizeof(buf) - len,
				" (dequeue pending)");
	}

	vchiq_dump(dump_context, buf, len + 1);
}

VCHIQ_STATE_T *
vchiq_get_state(void)
{

	if (g_state.remote == NULL)
		printk(KERN_ERR "%s: g_state.remote == NULL\n", __func__);
	else if (g_state.remote->initialised != 1)
		printk(KERN_NOTICE "%s: g_state.remote->initialised != 1 (%d)\n",
			__func__, g_state.remote->initialised);

	return ((g_state.remote != NULL) &&
		(g_state.remote->initialised == 1)) ? &g_state : NULL;
}

/*
 * Autosuspend related functionality
 */

int
vchiq_videocore_wanted(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	if (!arm_state)
		/* autosuspend not supported - always return wanted */
		return 1;
	else if (arm_state->blocked_count)
		return 1;
	else if (!arm_state->videocore_use_count)
		/* usage count zero - check for override unless we're forcing */
		if (arm_state->resume_blocked)
			return 0;
		else
			return vchiq_platform_videocore_wanted(state);
	else
		/* non-zero usage count - videocore still required */
		return 1;
}

static VCHIQ_STATUS_T
vchiq_keepalive_vchiq_callback(VCHIQ_REASON_T reason,
	VCHIQ_HEADER_T *header,
	VCHIQ_SERVICE_HANDLE_T service_user,
	void *bulk_user)
{
	vchiq_log_error(vchiq_susp_log_level,
		"%s callback reason %d", __func__, reason);
	return 0;
}

static int
vchiq_keepalive_thread_func(void *v)
{
	VCHIQ_STATE_T *state = (VCHIQ_STATE_T *) v;
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);

	VCHIQ_STATUS_T status;
	VCHIQ_INSTANCE_T instance;
	VCHIQ_SERVICE_HANDLE_T ka_handle;

	VCHIQ_SERVICE_PARAMS_T params = {
		.fourcc      = VCHIQ_MAKE_FOURCC('K', 'E', 'E', 'P'),
		.callback    = vchiq_keepalive_vchiq_callback,
		.version     = KEEPALIVE_VER,
		.version_min = KEEPALIVE_VER_MIN
	};

	status = vchiq_initialise(&instance);
	if (status != VCHIQ_SUCCESS) {
		vchiq_log_error(vchiq_susp_log_level,
			"%s vchiq_initialise failed %d", __func__, status);
		goto exit;
	}

	status = vchiq_connect(instance);
	if (status != VCHIQ_SUCCESS) {
		vchiq_log_error(vchiq_susp_log_level,
			"%s vchiq_connect failed %d", __func__, status);
		goto shutdown;
	}

	status = vchiq_add_service(instance, &params, &ka_handle);
	if (status != VCHIQ_SUCCESS) {
		vchiq_log_error(vchiq_susp_log_level,
			"%s vchiq_open_service failed %d", __func__, status);
		goto shutdown;
	}

	while (1) {
		long rc = 0, uc = 0;
		if (wait_for_completion_interruptible(&arm_state->ka_evt)
				!= 0) {
			vchiq_log_error(vchiq_susp_log_level,
				"%s interrupted", __func__);
			flush_signals(current);
			continue;
		}

		/* read and clear counters.  Do release_count then use_count to
		 * prevent getting more releases than uses */
		rc = atomic_xchg(&arm_state->ka_release_count, 0);
		uc = atomic_xchg(&arm_state->ka_use_count, 0);

		/* Call use/release service the requisite number of times.
		 * Process use before release so use counts don't go negative */
		while (uc--) {
			atomic_inc(&arm_state->ka_use_ack_count);
			status = vchiq_use_service(ka_handle);
			if (status != VCHIQ_SUCCESS) {
				vchiq_log_error(vchiq_susp_log_level,
					"%s vchiq_use_service error %d",
					__func__, status);
			}
		}
		while (rc--) {
			status = vchiq_release_service(ka_handle);
			if (status != VCHIQ_SUCCESS) {
				vchiq_log_error(vchiq_susp_log_level,
					"%s vchiq_release_service error %d",
					__func__, status);
			}
		}
	}

shutdown:
	vchiq_shutdown(instance);
exit:
	return 0;
}



VCHIQ_STATUS_T
vchiq_arm_init_state(VCHIQ_STATE_T *state, VCHIQ_ARM_STATE_T *arm_state)
{
	VCHIQ_STATUS_T status = VCHIQ_SUCCESS;

	if (arm_state) {
		rwlock_init(&arm_state->susp_res_lock);

		init_completion(&arm_state->ka_evt);
		atomic_set(&arm_state->ka_use_count, 0);
		atomic_set(&arm_state->ka_use_ack_count, 0);
		atomic_set(&arm_state->ka_release_count, 0);

		init_completion(&arm_state->vc_suspend_complete);

		init_completion(&arm_state->vc_resume_complete);
		/* Initialise to 'done' state.  We only want to block on resume
		 * completion while videocore is suspended. */
		set_resume_state(arm_state, VC_RESUME_RESUMED);

		init_completion(&arm_state->resume_blocker);
		/* Initialise to 'done' state.  We only want to block on this
		 * completion while resume is blocked */
		complete_all(&arm_state->resume_blocker);

		init_completion(&arm_state->blocked_blocker);
		/* Initialise to 'done' state.  We only want to block on this
		 * completion while things are waiting on the resume blocker */
		complete_all(&arm_state->blocked_blocker);

		arm_state->suspend_timer_timeout = SUSPEND_TIMER_TIMEOUT_MS;
		arm_state->suspend_timer_running = 0;
		init_timer(&arm_state->suspend_timer);
		arm_state->suspend_timer.data = (unsigned long)(state);
		arm_state->suspend_timer.function = suspend_timer_callback;

		arm_state->first_connect = 0;

	}
	return status;
}

/*
** Functions to modify the state variables;
**	set_suspend_state
**	set_resume_state
**
** There are more state variables than we might like, so ensure they remain in
** step.  Suspend and resume state are maintained separately, since most of
** these state machines can operate independently.  However, there are a few
** states where state transitions in one state machine cause a reset to the
** other state machine.  In addition, there are some completion events which
** need to occur on state machine reset and end-state(s), so these are also
** dealt with in these functions.
**
** In all states we set the state variable according to the input, but in some
** cases we perform additional steps outlined below;
**
** VC_SUSPEND_IDLE - Initialise the suspend completion at the same time.
**			The suspend completion is completed after any suspend
**			attempt.  When we reset the state machine we also reset
**			the completion.  This reset occurs when videocore is
**			resumed, and also if we initiate suspend after a suspend
**			failure.
**
** VC_SUSPEND_IN_PROGRESS - This state is considered the point of no return for
**			suspend - ie from this point on we must try to suspend
**			before resuming can occur.  We therefore also reset the
**			resume state machine to VC_RESUME_IDLE in this state.
**
** VC_SUSPEND_SUSPENDED - Suspend has completed successfully. Also call
**			complete_all on the suspend completion to notify
**			anything waiting for suspend to happen.
**
** VC_SUSPEND_REJECTED - Videocore rejected suspend. Videocore will also
**			initiate resume, so no need to alter resume state.
**			We call complete_all on the suspend completion to notify
**			of suspend rejection.
**
** VC_SUSPEND_FAILED - We failed to initiate videocore suspend.  We notify the
**			suspend completion and reset the resume state machine.
**
** VC_RESUME_IDLE - Initialise the resume completion at the same time.  The
**			resume completion is in it's 'done' state whenever
**			videcore is running.  Therfore, the VC_RESUME_IDLE state
**			implies that videocore is suspended.
**			Hence, any thread which needs to wait until videocore is
**			running can wait on this completion - it will only block
**			if videocore is suspended.
**
** VC_RESUME_RESUMED - Resume has completed successfully.  Videocore is running.
**			Call complete_all on the resume completion to unblock
**			any threads waiting for resume.	 Also reset the suspend
**			state machine to it's idle state.
**
** VC_RESUME_FAILED - Currently unused - no mechanism to fail resume exists.
*/

inline void
set_suspend_state(VCHIQ_ARM_STATE_T *arm_state,
	enum vc_suspend_status new_state)
{
	/* set the state in all cases */
	arm_state->vc_suspend_state = new_state;

	/* state specific additional actions */
	switch (new_state) {
	case VC_SUSPEND_FORCE_CANCELED:
		complete_all(&arm_state->vc_suspend_complete);
		break;
	case VC_SUSPEND_REJECTED:
		complete_all(&arm_state->vc_suspend_complete);
		break;
	case VC_SUSPEND_FAILED:
		complete_all(&arm_state->vc_suspend_complete);
		arm_state->vc_resume_state = VC_RESUME_RESUMED;
		complete_all(&arm_state->vc_resume_complete);
		break;
	case VC_SUSPEND_IDLE:
		INIT_COMPLETION(arm_state->vc_suspend_complete);
		break;
	case VC_SUSPEND_REQUESTED:
		break;
	case VC_SUSPEND_IN_PROGRESS:
		set_resume_state(arm_state, VC_RESUME_IDLE);
		break;
	case VC_SUSPEND_SUSPENDED:
		complete_all(&arm_state->vc_suspend_complete);
		break;
	default:
		BUG();
		break;
	}
}

inline void
set_resume_state(VCHIQ_ARM_STATE_T *arm_state,
	enum vc_resume_status new_state)
{
	/* set the state in all cases */
	arm_state->vc_resume_state = new_state;

	/* state specific additional actions */
	switch (new_state) {
	case VC_RESUME_FAILED:
		break;
	case VC_RESUME_IDLE:
		INIT_COMPLETION(arm_state->vc_resume_complete);
		break;
	case VC_RESUME_REQUESTED:
		break;
	case VC_RESUME_IN_PROGRESS:
		break;
	case VC_RESUME_RESUMED:
		complete_all(&arm_state->vc_resume_complete);
		set_suspend_state(arm_state, VC_SUSPEND_IDLE);
		break;
	default:
		BUG();
		break;
	}
}


/* should be called with the write lock held */
inline void
start_suspend_timer(VCHIQ_ARM_STATE_T *arm_state)
{
	del_timer(&arm_state->suspend_timer);
	arm_state->suspend_timer.expires = jiffies +
		msecs_to_jiffies(arm_state->
			suspend_timer_timeout);
	add_timer(&arm_state->suspend_timer);
	arm_state->suspend_timer_running = 1;
}

/* should be called with the write lock held */
static inline void
stop_suspend_timer(VCHIQ_ARM_STATE_T *arm_state)
{
	if (arm_state->suspend_timer_running) {
		del_timer(&arm_state->suspend_timer);
		arm_state->suspend_timer_running = 0;
	}
}

static inline int
need_resume(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	return (arm_state->vc_suspend_state > VC_SUSPEND_IDLE) &&
			(arm_state->vc_resume_state < VC_RESUME_REQUESTED) &&
			vchiq_videocore_wanted(state);
}

static int
block_resume(VCHIQ_ARM_STATE_T *arm_state)
{
	int status = VCHIQ_SUCCESS;
	const unsigned long timeout_val =
				msecs_to_jiffies(FORCE_SUSPEND_TIMEOUT_MS);
	int resume_count = 0;

	/* Allow any threads which were blocked by the last force suspend to
	 * complete if they haven't already.  Only give this one shot; if
	 * blocked_count is incremented after blocked_blocker is completed
	 * (which only happens when blocked_count hits 0) then those threads
	 * will have to wait until next time around */
	if (arm_state->blocked_count) {
		INIT_COMPLETION(arm_state->blocked_blocker);
		write_unlock_bh(&arm_state->susp_res_lock);
		vchiq_log_info(vchiq_susp_log_level, "%s wait for previously "
			"blocked clients", __func__);
		if (wait_for_completion_interruptible_timeout(
				&arm_state->blocked_blocker, timeout_val)
					<= 0) {
			vchiq_log_error(vchiq_susp_log_level, "%s wait for "
				"previously blocked clients failed" , __func__);
			status = VCHIQ_ERROR;
			write_lock_bh(&arm_state->susp_res_lock);
			goto out;
		}
		vchiq_log_info(vchiq_susp_log_level, "%s previously blocked "
			"clients resumed", __func__);
		write_lock_bh(&arm_state->susp_res_lock);
	}

	/* We need to wait for resume to complete if it's in process */
	while (arm_state->vc_resume_state != VC_RESUME_RESUMED &&
			arm_state->vc_resume_state > VC_RESUME_IDLE) {
		if (resume_count > 1) {
			status = VCHIQ_ERROR;
			vchiq_log_error(vchiq_susp_log_level, "%s waited too "
				"many times for resume" , __func__);
			goto out;
		}
		write_unlock_bh(&arm_state->susp_res_lock);
		vchiq_log_info(vchiq_susp_log_level, "%s wait for resume",
			__func__);
		if (wait_for_completion_interruptible_timeout(
				&arm_state->vc_resume_complete, timeout_val)
					<= 0) {
			vchiq_log_error(vchiq_susp_log_level, "%s wait for "
				"resume failed (%s)", __func__,
				resume_state_names[arm_state->vc_resume_state +
							VC_RESUME_NUM_OFFSET]);
			status = VCHIQ_ERROR;
			write_lock_bh(&arm_state->susp_res_lock);
			goto out;
		}
		vchiq_log_info(vchiq_susp_log_level, "%s resumed", __func__);
		write_lock_bh(&arm_state->susp_res_lock);
		resume_count++;
	}
	INIT_COMPLETION(arm_state->resume_blocker);
	arm_state->resume_blocked = 1;

out:
	return status;
}

static inline void
unblock_resume(VCHIQ_ARM_STATE_T *arm_state)
{
	complete_all(&arm_state->resume_blocker);
	arm_state->resume_blocked = 0;
}

/* Initiate suspend via slot handler. Should be called with the write lock
 * held */
VCHIQ_STATUS_T
vchiq_arm_vcsuspend(VCHIQ_STATE_T *state)
{
	VCHIQ_STATUS_T status = VCHIQ_ERROR;
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);
	status = VCHIQ_SUCCESS;


	switch (arm_state->vc_suspend_state) {
	case VC_SUSPEND_REQUESTED:
		vchiq_log_info(vchiq_susp_log_level, "%s: suspend already "
			"requested", __func__);
		break;
	case VC_SUSPEND_IN_PROGRESS:
		vchiq_log_info(vchiq_susp_log_level, "%s: suspend already in "
			"progress", __func__);
		break;

	default:
		/* We don't expect to be in other states, so log but continue
		 * anyway */
		vchiq_log_error(vchiq_susp_log_level,
			"%s unexpected suspend state %s", __func__,
			suspend_state_names[arm_state->vc_suspend_state +
						VC_SUSPEND_NUM_OFFSET]);
		/* fall through */
	case VC_SUSPEND_REJECTED:
	case VC_SUSPEND_FAILED:
		/* Ensure any idle state actions have been run */
		set_suspend_state(arm_state, VC_SUSPEND_IDLE);
		/* fall through */
	case VC_SUSPEND_IDLE:
		vchiq_log_info(vchiq_susp_log_level,
			"%s: suspending", __func__);
		set_suspend_state(arm_state, VC_SUSPEND_REQUESTED);
		/* kick the slot handler thread to initiate suspend */
		request_poll(state, NULL, 0);
		break;
	}

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit %d", __func__, status);
	return status;
}

void
vchiq_platform_check_suspend(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	int susp = 0;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	write_lock_bh(&arm_state->susp_res_lock);
	if (arm_state->vc_suspend_state == VC_SUSPEND_REQUESTED &&
			arm_state->vc_resume_state == VC_RESUME_RESUMED) {
		set_suspend_state(arm_state, VC_SUSPEND_IN_PROGRESS);
		susp = 1;
	}
	write_unlock_bh(&arm_state->susp_res_lock);

	if (susp)
		vchiq_platform_suspend(state);

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit", __func__);
	return;
}


static void
output_timeout_error(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	char service_err[50] = "";
	int vc_use_count = arm_state->videocore_use_count;
	int active_services = state->unused_service;
	int i;

	if (!arm_state->videocore_use_count) {
		snprintf(service_err, 50, " Videocore usecount is 0");
		goto output_msg;
	}
	for (i = 0; i < active_services; i++) {
		VCHIQ_SERVICE_T *service_ptr = state->services[i];
		if (service_ptr && service_ptr->service_use_count &&
			(service_ptr->srvstate != VCHIQ_SRVSTATE_FREE)) {
			snprintf(service_err, 50, " %c%c%c%c(%d) service has "
				"use count %d%s", VCHIQ_FOURCC_AS_4CHARS(
					service_ptr->base.fourcc),
				 service_ptr->client_id,
				 service_ptr->service_use_count,
				 service_ptr->service_use_count ==
					 vc_use_count ? "" : " (+ more)");
			break;
		}
	}

output_msg:
	vchiq_log_error(vchiq_susp_log_level,
		"timed out waiting for vc suspend (%d).%s",
		 arm_state->autosuspend_override, service_err);

}

/* Try to get videocore into suspended state, regardless of autosuspend state.
** We don't actually force suspend, since videocore may get into a bad state
** if we force suspend at a bad time.  Instead, we wait for autosuspend to
** determine a good point to suspend.  If this doesn't happen within 100ms we
** report failure.
**
** Returns VCHIQ_SUCCESS if videocore suspended successfully, VCHIQ_RETRY if
** videocore failed to suspend in time or VCHIQ_ERROR if interrupted.
*/
VCHIQ_STATUS_T
vchiq_arm_force_suspend(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	VCHIQ_STATUS_T status = VCHIQ_ERROR;
	long rc = 0;
	int repeat = -1;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	write_lock_bh(&arm_state->susp_res_lock);

	status = block_resume(arm_state);
	if (status != VCHIQ_SUCCESS)
		goto unlock;
	if (arm_state->vc_suspend_state == VC_SUSPEND_SUSPENDED) {
		/* Already suspended - just block resume and exit */
		vchiq_log_info(vchiq_susp_log_level, "%s already suspended",
			__func__);
		status = VCHIQ_SUCCESS;
		goto unlock;
	} else if (arm_state->vc_suspend_state <= VC_SUSPEND_IDLE) {
		/* initiate suspend immediately in the case that we're waiting
		 * for the timeout */
		stop_suspend_timer(arm_state);
		if (!vchiq_videocore_wanted(state)) {
			vchiq_log_info(vchiq_susp_log_level, "%s videocore "
				"idle, initiating suspend", __func__);
			status = vchiq_arm_vcsuspend(state);
		} else if (arm_state->autosuspend_override <
						FORCE_SUSPEND_FAIL_MAX) {
			vchiq_log_info(vchiq_susp_log_level, "%s letting "
				"videocore go idle", __func__);
			status = VCHIQ_SUCCESS;
		} else {
			vchiq_log_warning(vchiq_susp_log_level, "%s failed too "
				"many times - attempting suspend", __func__);
			status = vchiq_arm_vcsuspend(state);
		}
	} else {
		vchiq_log_info(vchiq_susp_log_level, "%s videocore suspend "
			"in progress - wait for completion", __func__);
		status = VCHIQ_SUCCESS;
	}

	/* Wait for suspend to happen due to system idle (not forced..) */
	if (status != VCHIQ_SUCCESS)
		goto unblock_resume;

	do {
		write_unlock_bh(&arm_state->susp_res_lock);

		rc = wait_for_completion_interruptible_timeout(
				&arm_state->vc_suspend_complete,
				msecs_to_jiffies(FORCE_SUSPEND_TIMEOUT_MS));

		write_lock_bh(&arm_state->susp_res_lock);
		if (rc < 0) {
			vchiq_log_warning(vchiq_susp_log_level, "%s "
				"interrupted waiting for suspend", __func__);
			status = VCHIQ_ERROR;
			goto unblock_resume;
		} else if (rc == 0) {
			if (arm_state->vc_suspend_state > VC_SUSPEND_IDLE) {
				/* Repeat timeout once if in progress */
				if (repeat < 0) {
					repeat = 1;
					continue;
				}
			}
			arm_state->autosuspend_override++;
			output_timeout_error(state);

			status = VCHIQ_RETRY;
			goto unblock_resume;
		}
	} while (0 < (repeat--));

	/* Check and report state in case we need to abort ARM suspend */
	if (arm_state->vc_suspend_state != VC_SUSPEND_SUSPENDED) {
		status = VCHIQ_RETRY;
		vchiq_log_error(vchiq_susp_log_level,
			"%s videocore suspend failed (state %s)", __func__,
			suspend_state_names[arm_state->vc_suspend_state +
						VC_SUSPEND_NUM_OFFSET]);
		/* Reset the state only if it's still in an error state.
		 * Something could have already initiated another suspend. */
		if (arm_state->vc_suspend_state < VC_SUSPEND_IDLE)
			set_suspend_state(arm_state, VC_SUSPEND_IDLE);

		goto unblock_resume;
	}

	/* successfully suspended - unlock and exit */
	goto unlock;

unblock_resume:
	/* all error states need to unblock resume before exit */
	unblock_resume(arm_state);

unlock:
	write_unlock_bh(&arm_state->susp_res_lock);

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit %d", __func__, status);
	return status;
}

void
vchiq_check_suspend(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	write_lock_bh(&arm_state->susp_res_lock);
	if (arm_state->vc_suspend_state != VC_SUSPEND_SUSPENDED &&
			arm_state->first_connect &&
			!vchiq_videocore_wanted(state)) {
		vchiq_arm_vcsuspend(state);
	}
	write_unlock_bh(&arm_state->susp_res_lock);

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit", __func__);
	return;
}


int
vchiq_arm_allow_resume(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	int resume = 0;
	int ret = -1;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	write_lock_bh(&arm_state->susp_res_lock);
	unblock_resume(arm_state);
	resume = vchiq_check_resume(state);
	write_unlock_bh(&arm_state->susp_res_lock);

	if (resume) {
		if (wait_for_completion_interruptible(
			&arm_state->vc_resume_complete) < 0) {
			vchiq_log_error(vchiq_susp_log_level,
				"%s interrupted", __func__);
			/* failed, cannot accurately derive suspend
			 * state, so exit early. */
			goto out;
		}
	}

	read_lock_bh(&arm_state->susp_res_lock);
	if (arm_state->vc_suspend_state == VC_SUSPEND_SUSPENDED) {
		vchiq_log_info(vchiq_susp_log_level,
				"%s: Videocore remains suspended", __func__);
	} else {
		vchiq_log_info(vchiq_susp_log_level,
				"%s: Videocore resumed", __func__);
		ret = 0;
	}
	read_unlock_bh(&arm_state->susp_res_lock);
out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit %d", __func__, ret);
	return ret;
}

/* This function should be called with the write lock held */
int
vchiq_check_resume(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	int resume = 0;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	if (need_resume(state)) {
		set_resume_state(arm_state, VC_RESUME_REQUESTED);
		request_poll(state, NULL, 0);
		resume = 1;
	}

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit", __func__);
	return resume;
}

void
vchiq_platform_check_resume(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	int res = 0;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	write_lock_bh(&arm_state->susp_res_lock);
	if (arm_state->wake_address == 0) {
		vchiq_log_info(vchiq_susp_log_level,
					"%s: already awake", __func__);
		goto unlock;
	}
	if (arm_state->vc_resume_state == VC_RESUME_IN_PROGRESS) {
		vchiq_log_info(vchiq_susp_log_level,
					"%s: already resuming", __func__);
		goto unlock;
	}

	if (arm_state->vc_resume_state == VC_RESUME_REQUESTED) {
		set_resume_state(arm_state, VC_RESUME_IN_PROGRESS);
		res = 1;
	} else
		vchiq_log_trace(vchiq_susp_log_level,
				"%s: not resuming (resume state %s)", __func__,
				resume_state_names[arm_state->vc_resume_state +
							VC_RESUME_NUM_OFFSET]);

unlock:
	write_unlock_bh(&arm_state->susp_res_lock);

	if (res)
		vchiq_platform_resume(state);

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit", __func__);
	return;

}



VCHIQ_STATUS_T
vchiq_use_internal(VCHIQ_STATE_T *state, VCHIQ_SERVICE_T *service,
		enum USE_TYPE_E use_type)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	VCHIQ_STATUS_T ret = VCHIQ_SUCCESS;
	char entity[16];
	int *entity_uc;
	int local_uc, local_entity_uc;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	if (use_type == USE_TYPE_VCHIQ) {
		sprintf(entity, "VCHIQ:   ");
		entity_uc = &arm_state->peer_use_count;
	} else if (service) {
		sprintf(entity, "%c%c%c%c:%03d",
			VCHIQ_FOURCC_AS_4CHARS(service->base.fourcc),
			service->client_id);
		entity_uc = &service->service_use_count;
	} else {
		vchiq_log_error(vchiq_susp_log_level, "%s null service "
				"ptr", __func__);
		ret = VCHIQ_ERROR;
		goto out;
	}

	write_lock_bh(&arm_state->susp_res_lock);
	while (arm_state->resume_blocked) {
		/* If we call 'use' while force suspend is waiting for suspend,
		 * then we're about to block the thread which the force is
		 * waiting to complete, so we're bound to just time out. In this
		 * case, set the suspend state such that the wait will be
		 * canceled, so we can complete as quickly as possible. */
		if (arm_state->resume_blocked && arm_state->vc_suspend_state ==
				VC_SUSPEND_IDLE) {
			set_suspend_state(arm_state, VC_SUSPEND_FORCE_CANCELED);
			break;
		}
		/* If suspend is already in progress then we need to block */
		if (!try_wait_for_completion(&arm_state->resume_blocker)) {
			/* Indicate that there are threads waiting on the resume
			 * blocker.  These need to be allowed to complete before
			 * a _second_ call to force suspend can complete,
			 * otherwise low priority threads might never actually
			 * continue */
			arm_state->blocked_count++;
			write_unlock_bh(&arm_state->susp_res_lock);
			vchiq_log_info(vchiq_susp_log_level, "%s %s resume "
				"blocked - waiting...", __func__, entity);
			if (wait_for_completion_killable(
					&arm_state->resume_blocker) != 0) {
				vchiq_log_error(vchiq_susp_log_level, "%s %s "
					"wait for resume blocker interrupted",
					__func__, entity);
				ret = VCHIQ_ERROR;
				write_lock_bh(&arm_state->susp_res_lock);
				arm_state->blocked_count--;
				write_unlock_bh(&arm_state->susp_res_lock);
				goto out;
			}
			vchiq_log_info(vchiq_susp_log_level, "%s %s resume "
				"unblocked", __func__, entity);
			write_lock_bh(&arm_state->susp_res_lock);
			if (--arm_state->blocked_count == 0)
				complete_all(&arm_state->blocked_blocker);
		}
	}

	stop_suspend_timer(arm_state);

	local_uc = ++arm_state->videocore_use_count;
	local_entity_uc = ++(*entity_uc);

	/* If there's a pending request which hasn't yet been serviced then
	 * just clear it.  If we're past VC_SUSPEND_REQUESTED state then
	 * vc_resume_complete will block until we either resume or fail to
	 * suspend */
	if (arm_state->vc_suspend_state <= VC_SUSPEND_REQUESTED)
		set_suspend_state(arm_state, VC_SUSPEND_IDLE);

	if ((use_type != USE_TYPE_SERVICE_NO_RESUME) && need_resume(state)) {
		set_resume_state(arm_state, VC_RESUME_REQUESTED);
		vchiq_log_info(vchiq_susp_log_level,
			"%s %s count %d, state count %d",
			__func__, entity, local_entity_uc, local_uc);
		request_poll(state, NULL, 0);
	} else
		vchiq_log_trace(vchiq_susp_log_level,
			"%s %s count %d, state count %d",
			__func__, entity, *entity_uc, local_uc);


	write_unlock_bh(&arm_state->susp_res_lock);

	/* Completion is in a done state when we're not suspended, so this won't
	 * block for the non-suspended case. */
	if (!try_wait_for_completion(&arm_state->vc_resume_complete)) {
		vchiq_log_info(vchiq_susp_log_level, "%s %s wait for resume",
			__func__, entity);
		if (wait_for_completion_killable(
				&arm_state->vc_resume_complete) != 0) {
			vchiq_log_error(vchiq_susp_log_level, "%s %s wait for "
				"resume interrupted", __func__, entity);
			ret = VCHIQ_ERROR;
			goto out;
		}
		vchiq_log_info(vchiq_susp_log_level, "%s %s resumed", __func__,
			entity);
	}

	if (ret == VCHIQ_SUCCESS) {
		VCHIQ_STATUS_T status = VCHIQ_SUCCESS;
		long ack_cnt = atomic_xchg(&arm_state->ka_use_ack_count, 0);
		while (ack_cnt && (status == VCHIQ_SUCCESS)) {
			/* Send the use notify to videocore */
			status = vchiq_send_remote_use_active(state);
			if (status == VCHIQ_SUCCESS)
				ack_cnt--;
			else
				atomic_add(ack_cnt,
					&arm_state->ka_use_ack_count);
		}
	}

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit %d", __func__, ret);
	return ret;
}

VCHIQ_STATUS_T
vchiq_release_internal(VCHIQ_STATE_T *state, VCHIQ_SERVICE_T *service)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	VCHIQ_STATUS_T ret = VCHIQ_SUCCESS;
	char entity[16];
	int *entity_uc;
	int local_uc, local_entity_uc;

	if (!arm_state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	if (service) {
		sprintf(entity, "%c%c%c%c:%03d",
			VCHIQ_FOURCC_AS_4CHARS(service->base.fourcc),
			service->client_id);
		entity_uc = &service->service_use_count;
	} else {
		sprintf(entity, "PEER:   ");
		entity_uc = &arm_state->peer_use_count;
	}

	write_lock_bh(&arm_state->susp_res_lock);
	if (!arm_state->videocore_use_count || !(*entity_uc)) {
		/* Don't use BUG_ON - don't allow user thread to crash kernel */
		WARN_ON(!arm_state->videocore_use_count);
		WARN_ON(!(*entity_uc));
		ret = VCHIQ_ERROR;
		goto unlock;
	}
	local_uc = --arm_state->videocore_use_count;
	local_entity_uc = --(*entity_uc);

	if (!vchiq_videocore_wanted(state)) {
		if (vchiq_platform_use_suspend_timer() &&
				!arm_state->resume_blocked) {
			/* Only use the timer if we're not trying to force
			 * suspend (=> resume_blocked) */
			start_suspend_timer(arm_state);
		} else {
			vchiq_log_info(vchiq_susp_log_level,
				"%s %s count %d, state count %d - suspending",
				__func__, entity, *entity_uc,
				arm_state->videocore_use_count);
			vchiq_arm_vcsuspend(state);
		}
	} else
		vchiq_log_trace(vchiq_susp_log_level,
			"%s %s count %d, state count %d",
			__func__, entity, *entity_uc,
			arm_state->videocore_use_count);

unlock:
	write_unlock_bh(&arm_state->susp_res_lock);

out:
	vchiq_log_trace(vchiq_susp_log_level, "%s exit %d", __func__, ret);
	return ret;
}

void
vchiq_on_remote_use(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);
	atomic_inc(&arm_state->ka_use_count);
	complete(&arm_state->ka_evt);
}

void
vchiq_on_remote_release(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);
	atomic_inc(&arm_state->ka_release_count);
	complete(&arm_state->ka_evt);
}

VCHIQ_STATUS_T
vchiq_use_service_internal(VCHIQ_SERVICE_T *service)
{
	return vchiq_use_internal(service->state, service, USE_TYPE_SERVICE);
}

VCHIQ_STATUS_T
vchiq_release_service_internal(VCHIQ_SERVICE_T *service)
{
	return vchiq_release_internal(service->state, service);
}

static void suspend_timer_callback(unsigned long context)
{
	VCHIQ_STATE_T *state = (VCHIQ_STATE_T *)context;
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	if (!arm_state)
		goto out;
	vchiq_log_info(vchiq_susp_log_level,
		"%s - suspend timer expired - check suspend", __func__);
	vchiq_check_suspend(state);
out:
	return;
}

VCHIQ_STATUS_T
vchiq_use_service_no_resume(VCHIQ_SERVICE_HANDLE_T handle)
{
	VCHIQ_STATUS_T ret = VCHIQ_ERROR;
	VCHIQ_SERVICE_T *service = find_service_by_handle(handle);
	if (service) {
		ret = vchiq_use_internal(service->state, service,
				USE_TYPE_SERVICE_NO_RESUME);
		unlock_service(service);
	}
	return ret;
}

VCHIQ_STATUS_T
vchiq_use_service(VCHIQ_SERVICE_HANDLE_T handle)
{
	VCHIQ_STATUS_T ret = VCHIQ_ERROR;
	VCHIQ_SERVICE_T *service = find_service_by_handle(handle);
	if (service) {
		ret = vchiq_use_internal(service->state, service,
				USE_TYPE_SERVICE);
		unlock_service(service);
	}
	return ret;
}

VCHIQ_STATUS_T
vchiq_release_service(VCHIQ_SERVICE_HANDLE_T handle)
{
	VCHIQ_STATUS_T ret = VCHIQ_ERROR;
	VCHIQ_SERVICE_T *service = find_service_by_handle(handle);
	if (service) {
		ret = vchiq_release_internal(service->state, service);
		unlock_service(service);
	}
	return ret;
}

void
vchiq_dump_service_use_state(VCHIQ_STATE_T *state)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	int i, j = 0;
	/* Only dump 64 services */
	static const int local_max_services = 64;
	/* If there's more than 64 services, only dump ones with
	 * non-zero counts */
	int only_nonzero = 0;
	static const char *nz = "<-- preventing suspend";

	enum vc_suspend_status vc_suspend_state;
	enum vc_resume_status  vc_resume_state;
	int peer_count;
	int vc_use_count;
	int active_services;
	struct service_data_struct {
		int fourcc;
		int clientid;
		int use_count;
	} service_data[local_max_services];

	if (!arm_state)
		return;

	read_lock_bh(&arm_state->susp_res_lock);
	vc_suspend_state = arm_state->vc_suspend_state;
	vc_resume_state  = arm_state->vc_resume_state;
	peer_count = arm_state->peer_use_count;
	vc_use_count = arm_state->videocore_use_count;
	active_services = state->unused_service;
	if (active_services > local_max_services)
		only_nonzero = 1;

	for (i = 0; (i < active_services) && (j < local_max_services); i++) {
		VCHIQ_SERVICE_T *service_ptr = state->services[i];
		if (!service_ptr)
			continue;

		if (only_nonzero && !service_ptr->service_use_count)
			continue;

		if (service_ptr->srvstate != VCHIQ_SRVSTATE_FREE) {
			service_data[j].fourcc = service_ptr->base.fourcc;
			service_data[j].clientid = service_ptr->client_id;
			service_data[j++].use_count = service_ptr->
							service_use_count;
		}
	}

	read_unlock_bh(&arm_state->susp_res_lock);

	vchiq_log_warning(vchiq_susp_log_level,
		"-- Videcore suspend state: %s --",
		suspend_state_names[vc_suspend_state + VC_SUSPEND_NUM_OFFSET]);
	vchiq_log_warning(vchiq_susp_log_level,
		"-- Videcore resume state: %s --",
		resume_state_names[vc_resume_state + VC_RESUME_NUM_OFFSET]);

	if (only_nonzero)
		vchiq_log_warning(vchiq_susp_log_level, "Too many active "
			"services (%d).  Only dumping up to first %d services "
			"with non-zero use-count", active_services,
			local_max_services);

	for (i = 0; i < j; i++) {
		vchiq_log_warning(vchiq_susp_log_level,
			"----- %c%c%c%c:%d service count %d %s",
			VCHIQ_FOURCC_AS_4CHARS(service_data[i].fourcc),
			service_data[i].clientid,
			service_data[i].use_count,
			service_data[i].use_count ? nz : "");
	}
	vchiq_log_warning(vchiq_susp_log_level,
		"----- VCHIQ use count count %d", peer_count);
	vchiq_log_warning(vchiq_susp_log_level,
		"--- Overall vchiq instance use count %d", vc_use_count);

	vchiq_dump_platform_use_state(state);
}

VCHIQ_STATUS_T
vchiq_check_service(VCHIQ_SERVICE_T *service)
{
	VCHIQ_ARM_STATE_T *arm_state;
	VCHIQ_STATUS_T ret = VCHIQ_ERROR;

	if (!service || !service->state)
		goto out;

	vchiq_log_trace(vchiq_susp_log_level, "%s", __func__);

	arm_state = vchiq_platform_get_arm_state(service->state);

	read_lock_bh(&arm_state->susp_res_lock);
	if (service->service_use_count)
		ret = VCHIQ_SUCCESS;
	read_unlock_bh(&arm_state->susp_res_lock);

	if (ret == VCHIQ_ERROR) {
		vchiq_log_error(vchiq_susp_log_level,
			"%s ERROR - %c%c%c%c:%d service count %d, "
			"state count %d, videocore suspend state %s", __func__,
			VCHIQ_FOURCC_AS_4CHARS(service->base.fourcc),
			service->client_id, service->service_use_count,
			arm_state->videocore_use_count,
			suspend_state_names[arm_state->vc_suspend_state +
						VC_SUSPEND_NUM_OFFSET]);
		vchiq_dump_service_use_state(service->state);
	}
out:
	return ret;
}

/* stub functions */
void vchiq_on_remote_use_active(VCHIQ_STATE_T *state)
{
	(void)state;
}

void vchiq_platform_conn_state_changed(VCHIQ_STATE_T *state,
	VCHIQ_CONNSTATE_T oldstate, VCHIQ_CONNSTATE_T newstate)
{
	VCHIQ_ARM_STATE_T *arm_state = vchiq_platform_get_arm_state(state);
	vchiq_log_info(vchiq_susp_log_level, "%d: %s->%s", state->id,
		get_conn_state_name(oldstate), get_conn_state_name(newstate));
	if (state->conn_state == VCHIQ_CONNSTATE_CONNECTED) {
		write_lock_bh(&arm_state->susp_res_lock);
		if (!arm_state->first_connect) {
			char threadname[10];
			arm_state->first_connect = 1;
			write_unlock_bh(&arm_state->susp_res_lock);
			snprintf(threadname, sizeof(threadname), "VCHIQka-%d",
				state->id);
			arm_state->ka_thread = kthread_create(
				&vchiq_keepalive_thread_func,
				(void *)state,
				threadname);
			if (arm_state->ka_thread == NULL) {
				vchiq_log_error(vchiq_susp_log_level,
					"vchiq: FATAL: couldn't create thread %s",
					threadname);
			} else {
				wake_up_process(arm_state->ka_thread);
			}
		} else
			write_unlock_bh(&arm_state->susp_res_lock);
	}
}


/****************************************************************************
*
*   vchiq_init - called when the module is loaded.
*
***************************************************************************/

static int
vchiq_probe(struct platform_device *pdev)
{
	int err;

	err = vchiq_platform_init(pdev, &g_state);
	if (err != 0)
		goto failed_platform_init;

	vchiq_log_info(vchiq_arm_log_level,
		"vchiq: initialised - version %d (min %d)",
		VCHIQ_VERSION, VCHIQ_VERSION_MIN);

	return 0;

failed_platform_init:
	vchiq_log_warning(vchiq_arm_log_level, "could not load vchiq");
	return err;
}

/****************************************************************************
*
*   vchiq_exit - called when the module is unloaded.
*
***************************************************************************/

static int
vchiq_remove(struct platform_device *pdev)
{
	vchiq_platform_exit(pdev, &g_state);

	return 0;
}

static const struct of_device_id vchiq_of_match[] = {
        { .compatible = "brcm,bcm2835-vchiq", },
        {},
};
MODULE_DEVICE_TABLE(of, vchiq_of_match);

static struct platform_driver vchiq_driver = {
        .driver = {
                .name = "vchiq",
                .owner = THIS_MODULE,
                .of_match_table = vchiq_of_match,
        },
        .probe          = vchiq_probe,
        .remove         = vchiq_remove,
};
module_platform_driver(vchiq_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
