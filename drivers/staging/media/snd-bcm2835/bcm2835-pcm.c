/*
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

#include <linux/interrupt.h>
#include <linux/slab.h>

#include "bcm2835.h"

/* hardware definition */
static struct snd_pcm_hardware snd_bcm2835_playback_hw = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 32 * 1024,	/* Needs to be less than audioplay
					 * buffer size */
	.period_bytes_min =  4 * 1024,
	.period_bytes_max = 32 * 1024,
	.periods_min = 1,
	.periods_max = 32,
};

static void snd_bcm2835_playback_free(struct snd_pcm_runtime *runtime)
{
	kfree(runtime->private_data);
	runtime->private_data = NULL;
}

static irqreturn_t bcm2835_playback_fifo_irq(int irq, void *dev_id)
{
	struct bcm2835_alsa_stream *alsa_stream =
		(struct bcm2835_alsa_stream *)dev_id;
	struct device *dev = alsa_stream->chip->card->dev;
	uint32_t consumed = 0;
	int new_period = 0;

	if (alsa_stream->open)
		consumed = bcm2835_audio_retrieve_buffers(alsa_stream);

	/* We get called only if playback was triggered, So, the number of
	 * buffers we retrieve in each iteration are the buffers that have
	 * been played out already */

	if (alsa_stream->period_size) {
		if ((alsa_stream->pos / alsa_stream->period_size) !=
		    ((alsa_stream->pos + consumed) / alsa_stream->period_size))
			new_period = 1;
	}
	if (alsa_stream->buffer_size) {
		alsa_stream->pos += consumed & ~(1<<30);
		alsa_stream->pos %= alsa_stream->buffer_size;
	}

	if (alsa_stream->substream) {
		if (new_period)
			snd_pcm_period_elapsed(alsa_stream->substream);
	} else {
		dev_err(dev, "unexpected NULL substream\n");
	}

	return IRQ_HANDLED;
}

/* open callback */
static int snd_bcm2835_playback_open(struct snd_pcm_substream *substream)
{
	struct bcm2835_chip *chip = snd_pcm_substream_chip(substream);
	struct device *dev = chip->card->dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream;
	int idx;
	int err;

	idx = substream->number;

	if (idx > MAX_SUBSTREAMS) {
		dev_err(dev, "substream %d doesn't exist\n", idx);
		err = -ENODEV;
		goto out;
	}

	/* Check if we are ready */
	if (!(chip->avail_substreams & (1 << idx))) {
		/* We are not ready yet */
		dev_err(dev, "substream(%d) device is not ready yet\n", idx);
		err = -EAGAIN;
		goto out;
	}

	alsa_stream = kzalloc(sizeof(struct bcm2835_alsa_stream), GFP_KERNEL);
	if (alsa_stream == NULL)
		return -ENOMEM;

	/* Initialise alsa_stream */
	alsa_stream->chip = chip;
	alsa_stream->substream = substream;
	alsa_stream->idx = idx;
	chip->alsa_stream[idx] = alsa_stream;

	sema_init(&alsa_stream->buffers_update_sem, 0);
	sema_init(&alsa_stream->control_sem, 0);
	spin_lock_init(&alsa_stream->lock);

	/* Enabled in start trigger, called on each "fifo irq" after that */
	alsa_stream->enable_fifo_irq = 0;
	alsa_stream->fifo_irq_handler = bcm2835_playback_fifo_irq;

	runtime->private_data = alsa_stream;
	runtime->private_free = snd_bcm2835_playback_free;
	runtime->hw = snd_bcm2835_playback_hw;
	/* minimum 16 bytes alignment (for vchiq bulk transfers) */
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				   16);

	err = bcm2835_audio_open(alsa_stream);
	if (err != 0) {
		kfree(alsa_stream);
		return err;
	}

	alsa_stream->open = 1;
	alsa_stream->draining = 1;

out:
	return err;
}

/* close callback */
static int snd_bcm2835_playback_close(struct snd_pcm_substream *substream)
{
	struct bcm2835_chip *chip = snd_pcm_substream_chip(substream);
	struct device *dev = chip->card->dev;

	/* the hardware-specific codes will be here */

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream = runtime->private_data;

	/*
	 *Call stop if it's still running. This happens when app
	 *is force killed and we don't get a stop trigger.
	 */
	if (alsa_stream->running) {
		int err;
		err = bcm2835_audio_stop(alsa_stream);
		alsa_stream->running = 0;
		if (err != 0)
			dev_err(dev, "Failed to STOP alsa device\n");
	}

	alsa_stream->period_size = 0;
	alsa_stream->buffer_size = 0;

	if (alsa_stream->open) {
		alsa_stream->open = 0;
		bcm2835_audio_close(alsa_stream);
	}
	if (alsa_stream->chip)
		alsa_stream->chip->alsa_stream[alsa_stream->idx] = NULL;
	/*
	 *Do not free up alsa_stream here, it will be freed up by
	 *runtime->private_free callback we registered in *_open above
	 */

	return 0;
}

/* hw_params callback */
static int snd_bcm2835_pcm_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct bcm2835_chip *chip = snd_pcm_substream_chip(substream);
	struct device *dev = chip->card->dev;
	int err;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream =
	    (struct bcm2835_alsa_stream *) runtime->private_data;

	err = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (err < 0) {
		dev_err(dev, "pcm_lib_malloc failed to allocated pages for buffers\n");
		return err;
	}

	err = bcm2835_audio_set_params(alsa_stream, params_channels(params),
				       params_rate(params),
				       snd_pcm_format_width(params_format
							    (params)));
	if (err < 0)
		dev_err(dev, "error setting hw params\n");

	bcm2835_audio_setup(alsa_stream);

	/* in preparation of the stream, set the controls (volume level)
	 * of the stream */
	bcm2835_audio_set_ctls(alsa_stream->chip);

	return err;
}

/* hw_free callback */
static int snd_bcm2835_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

/* prepare callback */
static int snd_bcm2835_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream = runtime->private_data;

	alsa_stream->buffer_size = snd_pcm_lib_buffer_bytes(substream);
	alsa_stream->period_size = snd_pcm_lib_period_bytes(substream);
	alsa_stream->pos = 0;

	return 0;
}

/* trigger callback */
static int snd_bcm2835_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct bcm2835_chip *chip = snd_pcm_substream_chip(substream);
	struct device *dev = chip->card->dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream = runtime->private_data;
	int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (!alsa_stream->running) {
			err = bcm2835_audio_start(alsa_stream);
			if (err == 0) {
				alsa_stream->running = 1;
				alsa_stream->draining = 1;
			} else {
				dev_err(dev, "Failed to start device\n");
			}
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (runtime->status->state == SNDRV_PCM_STATE_DRAINING)
			alsa_stream->draining = 1;
		else
			alsa_stream->draining = 0;
		if (alsa_stream->running) {
			err = bcm2835_audio_stop(alsa_stream);
			if (err != 0)
				dev_err(dev, "Failed to stop device\n");
			alsa_stream->running = 0;
		}
		break;
	default:
		err = -EINVAL;
	}

	return err;
}

/* pointer callback */
static snd_pcm_uframes_t
snd_bcm2835_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream = runtime->private_data;

	return bytes_to_frames(runtime, alsa_stream->pos);
}

static int snd_bcm2835_pcm_copy(struct snd_pcm_substream *substream,
				int channel, snd_pcm_uframes_t pos, void *src,
				snd_pcm_uframes_t count)
{
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm2835_alsa_stream *alsa_stream = runtime->private_data;

	ret = bcm2835_audio_write(alsa_stream,
		frames_to_bytes(runtime, count), src);
	return ret;
}

static int snd_bcm2835_pcm_lib_ioctl(struct snd_pcm_substream *substream,
				     unsigned int cmd, void *arg)
{
	int ret = snd_pcm_lib_ioctl(substream, cmd, arg);
	return ret;
}

/* operators */
static struct snd_pcm_ops snd_bcm2835_playback_ops = {
	.open = snd_bcm2835_playback_open,
	.close = snd_bcm2835_playback_close,
	.ioctl = snd_bcm2835_pcm_lib_ioctl,
	.hw_params = snd_bcm2835_pcm_hw_params,
	.hw_free = snd_bcm2835_pcm_hw_free,
	.prepare = snd_bcm2835_pcm_prepare,
	.trigger = snd_bcm2835_pcm_trigger,
	.pointer = snd_bcm2835_pcm_pointer,
	.copy = snd_bcm2835_pcm_copy,
};

/* create a pcm device */
int snd_bcm2835_new_pcm(struct bcm2835_chip *chip)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(chip->card, "bcm2835 ALSA", 0, MAX_SUBSTREAMS,
								0, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = chip;
	strcpy(pcm->name, "bcm2835 ALSA");
	chip->pcm = pcm;
	chip->dest = AUDIO_DEST_AUTO;
	chip->volume = alsa2chip(0);
	chip->mute = CTRL_VOL_UNMUTE;	/*disable mute on startup */
	/* set operators */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_bcm2835_playback_ops);

	/* pre-allocation of buffers */
	/* NOTE: this may fail */
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL), 64 * 1024,
					      64 * 1024);

	return 0;
}
