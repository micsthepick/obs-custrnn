/*
OBS CustRNN
Copyright (C) 2023 micsthepick micksthepick.bots@gmail.com

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>
*/

#include <stdint.h>
#include <inttypes.h>

#include <util/circlebuf.h>
#include <util/threading.h>
#include <obs-module.h>

#ifdef _MSC_VER
#define ssize_t intptr_t
#endif

#include "rnnoise/include/rnnoise.h"
#include <media-io/audio-resampler.h>

#define MAX_PREPROC_CHANNELS 8

/* RNNoise constants, these can't be changed */
#define RNNOISE_SAMPLE_RATE 48000
#define RNNOISE_FRAME_SIZE 480

/* If the following constant changes, RNNoise breaks */
#define BUFFER_SIZE_MSEC 10

struct noise_suppress_data {
	obs_source_t *context;

	uint64_t last_timestamp;
	uint64_t latency;

	size_t frames;
	size_t channels;

	struct circlebuf info_buffer;
	struct circlebuf input_buffers[MAX_PREPROC_CHANNELS];
	struct circlebuf output_buffers[MAX_PREPROC_CHANNELS];

	bool has_mono_src;
	volatile bool reinit_done;

	/* RNNoise state */
	DenoiseState *rnn_states[MAX_PREPROC_CHANNELS];

	/* Resampler */
	audio_resampler_t *rnn_resampler;
	audio_resampler_t *rnn_resampler_back;

	/* PCM buffers */
	float *copy_buffers[MAX_PREPROC_CHANNELS];
	float *rnn_segment_buffers[MAX_PREPROC_CHANNELS];

	/* output data */
	struct obs_audio_data output_audio;
	DARRAY(float) output_data;
};

static const char *noise_suppress_cust_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return obs_module_text("NoiseSuppressCust");
}

static void noise_suppress_cust_destroy(void *data)
{
	struct noise_suppress_data *ng = data;

	for (size_t i = 0; i < ng->channels; i++) {
		rnnoise_destroy(ng->rnn_states[i]);

		circlebuf_free(&ng->input_buffers[i]);
		circlebuf_free(&ng->output_buffers[i]);
	}

	bfree(ng->rnn_segment_buffers[0]);

	if (ng->rnn_resampler) {
		audio_resampler_destroy(ng->rnn_resampler);
		audio_resampler_destroy(ng->rnn_resampler_back);
	}

	bfree(ng->copy_buffers[0]);
	circlebuf_free(&ng->info_buffer);
	da_free(ng->output_data);
	bfree(ng);
}

static inline void alloc_channel(struct noise_suppress_data *ng,
				 uint32_t sample_rate, size_t channel,
				 size_t frames)
{
	UNUSED_PARAMETER(sample_rate);
	ng->rnn_states[channel] = rnnoise_create(NULL);
	circlebuf_reserve(&ng->input_buffers[channel], frames * sizeof(float));
	circlebuf_reserve(&ng->output_buffers[channel], frames * sizeof(float));
}

static inline enum speaker_layout convert_speaker_layout(uint8_t channels)
{
	switch (channels) {
	case 0:
		return SPEAKERS_UNKNOWN;
	case 1:
		return SPEAKERS_MONO;
	case 2:
		return SPEAKERS_STEREO;
	case 3:
		return SPEAKERS_2POINT1;
	case 4:
		return SPEAKERS_4POINT0;
	case 5:
		return SPEAKERS_4POINT1;
	case 6:
		return SPEAKERS_5POINT1;
	case 8:
		return SPEAKERS_7POINT1;
	default:
		return SPEAKERS_UNKNOWN;
	}
}

static void noise_suppress_cust_update(void *data, obs_data_t *settings)
{
	UNUSED_PARAMETER(settings);
	struct noise_suppress_data *ng = data;

	uint32_t sample_rate = audio_output_get_sample_rate(obs_get_audio());
	size_t channels = audio_output_get_channels(obs_get_audio());
	size_t frames = (size_t)sample_rate / (1000 / BUFFER_SIZE_MSEC);

	ng->latency = 1000000000LL / (1000 / BUFFER_SIZE_MSEC);
	/* Process 10 millisecond segments to keep latency low. */
	/* Also RNNoise only supports buffers of this exact size. */
	ng->frames = frames;
	ng->channels = channels;

	if (ng->rnn_states[0])
		return;

	/* One speex/rnnoise state for each channel (limit 2) */
	ng->copy_buffers[0] = bmalloc(frames * channels * sizeof(float));
	ng->rnn_segment_buffers[0] =
		bmalloc(RNNOISE_FRAME_SIZE * channels * sizeof(float));
	for (size_t c = 1; c < channels; ++c) {
		ng->copy_buffers[c] = ng->copy_buffers[c - 1] + frames;
		ng->rnn_segment_buffers[c] =
			ng->rnn_segment_buffers[c - 1] + RNNOISE_FRAME_SIZE;
	}

	for (size_t i = 0; i < channels; i++)
		alloc_channel(ng, sample_rate, i, frames);

	if (sample_rate == RNNOISE_SAMPLE_RATE) {
		ng->rnn_resampler = NULL;
		ng->rnn_resampler_back = NULL;
	} else {
		struct resample_info src, dst;
		src.samples_per_sec = sample_rate;
		src.format = AUDIO_FORMAT_FLOAT_PLANAR;
		src.speakers = convert_speaker_layout((uint8_t)channels);

		dst.samples_per_sec = RNNOISE_SAMPLE_RATE;
		dst.format = AUDIO_FORMAT_FLOAT_PLANAR;
		dst.speakers = convert_speaker_layout((uint8_t)channels);

		ng->rnn_resampler = audio_resampler_create(&dst, &src);
		ng->rnn_resampler_back = audio_resampler_create(&src, &dst);
	}
}

static void *noise_suppress_cust_create(obs_data_t *settings,
					obs_source_t *filter)
{
	UNUSED_PARAMETER(settings);
	struct noise_suppress_data *ng =
		bzalloc(sizeof(struct noise_suppress_data));

	ng->context = filter;

	noise_suppress_cust_update(ng, settings);
	return ng;
}

static inline void process_rnnoise(struct noise_suppress_data *ng)
{
	/* Adjust signal level to what RNNoise expects, resample if necessary */
	if (ng->rnn_resampler) {
		float *output[MAX_PREPROC_CHANNELS];
		uint32_t out_frames;
		uint64_t ts_offset;
		audio_resampler_resample(ng->rnn_resampler, (uint8_t **)output,
					 &out_frames, &ts_offset,
					 (const uint8_t **)ng->copy_buffers,
					 (uint32_t)ng->frames);

		for (size_t i = 0; i < ng->channels; i++) {
			for (ssize_t j = 0, k = (ssize_t)out_frames -
						RNNOISE_FRAME_SIZE;
			     j < RNNOISE_FRAME_SIZE; ++j, ++k) {
				if (k >= 0) {
					ng->rnn_segment_buffers[i][j] =
						output[i][k] * 32768.0f;
				} else {
					ng->rnn_segment_buffers[i][j] = 0;
				}
			}
		}
	} else {
		for (size_t i = 0; i < ng->channels; i++) {
			for (size_t j = 0; j < RNNOISE_FRAME_SIZE; ++j) {
				ng->rnn_segment_buffers[i][j] =
					ng->copy_buffers[i][j] * 32768.0f;
			}
		}
	}

	/* Execute */
	for (size_t i = 0; i < ng->channels; i++) {
		rnnoise_process_frame(ng->rnn_states[i],
				      ng->rnn_segment_buffers[i],
				      ng->rnn_segment_buffers[i]);
	}

	/* Revert signal level adjustment, resample back if necessary */
	if (ng->rnn_resampler) {
		float *output[MAX_PREPROC_CHANNELS];
		uint32_t out_frames;
		uint64_t ts_offset;
		audio_resampler_resample(
			ng->rnn_resampler_back, (uint8_t **)output, &out_frames,
			&ts_offset, (const uint8_t **)ng->rnn_segment_buffers,
			RNNOISE_FRAME_SIZE);

		for (size_t i = 0; i < ng->channels; i++) {
			for (ssize_t j = 0,
				     k = (ssize_t)out_frames - ng->frames;
			     j < (ssize_t)ng->frames; ++j, ++k) {
				if (k >= 0) {
					ng->copy_buffers[i][j] =
						output[i][k] / 32768.0f;
				} else {
					ng->copy_buffers[i][j] = 0;
				}
			}
		}
	} else {
		for (size_t i = 0; i < ng->channels; i++) {
			for (size_t j = 0; j < RNNOISE_FRAME_SIZE; ++j) {
				ng->copy_buffers[i][j] =
					ng->rnn_segment_buffers[i][j] /
					32768.0f;
			}
		}
	}
}

static inline void process(struct noise_suppress_data *ng)
{
	/* Pop from input circlebuf */
	for (size_t i = 0; i < ng->channels; i++)
		circlebuf_pop_front(&ng->input_buffers[i], ng->copy_buffers[i],
				    ng->frames * sizeof(float));

	process_rnnoise(ng);

	/* Push to output circlebuf */
	for (size_t i = 0; i < ng->channels; i++)
		circlebuf_push_back(&ng->output_buffers[i], ng->copy_buffers[i],
				    ng->frames * sizeof(float));
}

struct ng_audio_info {
	uint32_t frames;
	uint64_t timestamp;
};

static inline void clear_circlebuf(struct circlebuf *buf)
{
	circlebuf_pop_front(buf, NULL, buf->size);
}

static void reset_data(struct noise_suppress_data *ng)
{
	for (size_t i = 0; i < ng->channels; i++) {
		clear_circlebuf(&ng->input_buffers[i]);
		clear_circlebuf(&ng->output_buffers[i]);
	}

	clear_circlebuf(&ng->info_buffer);
}

static struct obs_audio_data *
noise_suppress_cust_filter_audio(void *data, struct obs_audio_data *audio)
{
	struct noise_suppress_data *ng = data;
	struct ng_audio_info info;
	size_t segment_size = ng->frames * sizeof(float);
	size_t out_size;
	obs_source_t *parent = obs_filter_get_parent(ng->context);
	enum speaker_layout layout = obs_source_get_speaker_layout(parent);
	ng->has_mono_src = layout == SPEAKERS_MONO && ng->channels == 2;

	if (!ng->rnn_states[0])
		return audio;

	/* -----------------------------------------------
	 * if timestamp has dramatically changed, consider it a new stream of
	 * audio data.  clear all circular buffers to prevent old audio data
	 * from being processed as part of the new data. */
	if (ng->last_timestamp) {
		int64_t diff = llabs((int64_t)ng->last_timestamp -
				     (int64_t)audio->timestamp);

		if (diff > 1000000000LL)
			reset_data(ng);
	}

	ng->last_timestamp = audio->timestamp;

	/* -----------------------------------------------
	 * push audio packet info (timestamp/frame count) to info circlebuf */
	info.frames = audio->frames;
	info.timestamp = audio->timestamp;
	circlebuf_push_back(&ng->info_buffer, &info, sizeof(info));

	/* -----------------------------------------------
	 * push back current audio data to input circlebuf */
	for (size_t i = 0; i < ng->channels; i++)
		circlebuf_push_back(&ng->input_buffers[i], audio->data[i],
				    audio->frames * sizeof(float));

	/* -----------------------------------------------
	 * pop/process each 10ms segments, push back to output circlebuf */
	while (ng->input_buffers[0].size >= segment_size)
		process(ng);

	/* -----------------------------------------------
	 * peek front of info circlebuf, check to see if we have enough to
	 * pop the expected packet size, if not, return null */
	memset(&info, 0, sizeof(info));
	circlebuf_peek_front(&ng->info_buffer, &info, sizeof(info));
	out_size = info.frames * sizeof(float);

	if (ng->output_buffers[0].size < out_size)
		return NULL;

	/* -----------------------------------------------
	 * if there's enough audio data buffered in the output circlebuf,
	 * pop and return a packet */
	circlebuf_pop_front(&ng->info_buffer, NULL, sizeof(info));
	da_resize(ng->output_data, out_size * ng->channels);

	for (size_t i = 0; i < ng->channels; i++) {
		ng->output_audio.data[i] =
			(uint8_t *)&ng->output_data.array[i * out_size];

		circlebuf_pop_front(&ng->output_buffers[i],
				    ng->output_audio.data[i], out_size);
	}

	ng->output_audio.frames = info.frames;
	ng->output_audio.timestamp = info.timestamp - ng->latency;
	return &ng->output_audio;
}

struct obs_source_info noise_suppress_filter_cust = {
	.id = "noise_suppress_filter_cust",
	.type = OBS_SOURCE_TYPE_FILTER,
	.output_flags = OBS_SOURCE_AUDIO,
	.get_name = noise_suppress_cust_name,
	.create = noise_suppress_cust_create,
	.destroy = noise_suppress_cust_destroy,
	.update = noise_suppress_cust_update,
	.filter_audio = noise_suppress_cust_filter_audio,
};