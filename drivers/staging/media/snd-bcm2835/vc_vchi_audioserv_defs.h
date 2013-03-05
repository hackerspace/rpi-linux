/*
 *
 * Copyright 2011 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 */

#ifndef _VC_AUDIO_DEFS_H_
#define _VC_AUDIO_DEFS_H_

#define VC_AUDIOSERV_MIN_VER 1
#define VC_AUDIOSERV_VER 2

/* FourCC code used for VCHI connection */
#define VC_AUDIO_SERVER_NAME  MAKE_FOURCC("AUDS")

/* Maximum message length */
#define VC_AUDIO_MAX_MSG_LEN  (sizeof(VC_AUDIO_MSG_T))

/* List of screens that are currently supported */
/* All message types supported for HOST->VC direction */
enum {
	VC_AUDIO_MSG_TYPE_RESULT,	/* Generic result */
	VC_AUDIO_MSG_TYPE_COMPLETE,	/* Generic result */
	VC_AUDIO_MSG_TYPE_CONFIG,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_CONTROL,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_OPEN,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_CLOSE,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_START,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_STOP,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_WRITE,	/* Configure audio */
	VC_AUDIO_MSG_TYPE_MAX
};

/* configure the audio */
struct vc_audio_config {
	uint32_t channels;
	uint32_t samplerate;
	uint32_t bps;
};

struct vc_audio_control {
	uint32_t volume;
	uint32_t dest;
};

/* audio */
struct vc_audio_open {
	uint32_t dummy;
};

/* audio */
struct vc_audio_close {
	uint32_t dummy;
};

/* audio */
struct vc_audio_start {
	uint32_t dummy;
};

/* audio */
struct vc_audio_stop {
	uint32_t draining;
};

/* configure the write audio samples */
struct vc_audio_write {
	uint32_t count;		/* in bytes */
	void *callback;
	void *cookie;
	uint16_t silence;
	uint16_t max_packet;
};

/* Generic result for a request (VC->HOST) */
struct vc_audio_result {
	int32_t success;	/* Success value */

};

/* Generic result for a request (VC->HOST) */
struct vc_audio_complete {
	int32_t count;		/* Success value */
	void *callback;
	void *cookie;
};

/* Message header for all messages in HOST->VC direction */
struct vc_audio_msg {
	int32_t type;		/* Message type (VC_AUDIO_MSG_TYPE) */
	union {
		struct vc_audio_config config;
		struct vc_audio_control control;
		struct vc_audio_open open;
		struct vc_audio_close close;
		struct vc_audio_start start;
		struct vc_audio_stop stop;
		struct vc_audio_write write;
		struct vc_audio_result result;
		struct vc_audio_complete complete;
	} u;
};

#endif /* _VC_AUDIO_DEFS_H_ */
