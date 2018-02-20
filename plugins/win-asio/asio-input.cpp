/*
Copyright (C) 2017 by pkv <pkv.stream@gmail.com>, andersama <anderson.john.alexander@gmail.com>

Based on Pulse Input plugin by Leonhard Oelke.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <util/bmem.h>
#include <util/platform.h>
#include <util/threading.h>
#include <obs-module.h>
#include <obs-frontend-api.h>
#include <vector>
#include <stdio.h>
#include <string>
#include <windows.h>
#include "portaudio.h"
#include "pa_asio.h"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("win-asio", "en-US")

#define blog(level, msg, ...) blog(level, "asio-input: " msg, ##__VA_ARGS__)

#define NSEC_PER_SEC  1000000000LL

#define TEXT_BUFFER_SIZE                obs_module_text("BufferSize")
#define TEXT_BUFFER_64_SAMPLES          obs_module_text("64_samples")
#define TEXT_BUFFER_128_SAMPLES         obs_module_text("128_samples")
#define TEXT_BUFFER_256_SAMPLES         obs_module_text("256_samples")
#define TEXT_BUFFER_512_SAMPLES         obs_module_text("512_samples")
#define TEXT_BUFFER_1024_SAMPLES        obs_module_text("1024_samples")
#define TEXT_BITDEPTH                   obs_module_text("BitDepth")


typedef struct PaAsioDeviceInfo
{
	PaDeviceInfo commonDeviceInfo;
	long minBufferSize;
	long maxBufferSize;
	long preferredBufferSize;
	long bufferGranularity;

}
PaAsioDeviceInfo;

struct asio_data {
	obs_source_t *source;

	/*asio device and info */
	const char *device;
	uint8_t device_index;
	PaAsioDeviceInfo *info;
	PaStream *stream;
	obs_data_t *settings;

	audio_format BitDepth; // 16bit or 32 bit
	int SampleRate;          //44100 or 48000 Hz
	uint16_t BufferSize;     // number of samples in buffer
	uint64_t first_ts;       //first timestamp

	/* channels info */
	int channels; //total number of input channels
	int output_channels; // number of output channels of device (not used)
	int recorded_channels; // number of channels passed from device (including muted) to OBS; is at most 8
	int route[MAX_AUDIO_CHANNELS]; // stores the channel re-ordering info
};

/* global RtAudio */
//RtAudio adc;

/* ======================================================================= */
/* conversion between portaudio and obs */

enum audio_format portaudio_to_obs_audio_format(PaSampleFormat format)
{
	switch (format) {
	case paInt16:   return AUDIO_FORMAT_16BIT;
	case paInt32:   return AUDIO_FORMAT_32BIT;
	case paFloat32:  return AUDIO_FORMAT_FLOAT;
	default:               break;
	}

	return AUDIO_FORMAT_UNKNOWN;
}

enum audio_format get_planar_format(audio_format format)
{
	if (is_audio_planar(format))
		return format;

	switch (format) {
	case AUDIO_FORMAT_U8BIT: return AUDIO_FORMAT_U8BIT_PLANAR;
	case AUDIO_FORMAT_16BIT: return AUDIO_FORMAT_16BIT_PLANAR;
	case AUDIO_FORMAT_32BIT: return AUDIO_FORMAT_32BIT_PLANAR;
	case AUDIO_FORMAT_FLOAT: return AUDIO_FORMAT_FLOAT_PLANAR;
		//should NEVER get here
	default: return AUDIO_FORMAT_UNKNOWN;
	}
}

int bytedepth_format(audio_format format)
{
	return (int)get_audio_bytes_per_channel(format);
}

int bytedepth_format(PaSampleFormat format) {
	return bytedepth_format(portaudio_to_obs_audio_format(format));
}

PaSampleFormat obs_to_portaudio_audio_format(audio_format format)
{
	switch (format) {
	case AUDIO_FORMAT_U8BIT:
	case AUDIO_FORMAT_U8BIT_PLANAR:
		return paUInt8;

	case AUDIO_FORMAT_16BIT:
	case AUDIO_FORMAT_16BIT_PLANAR:
		return paInt16;
		// obs doesn't have 24 bit
	case AUDIO_FORMAT_32BIT:
	case AUDIO_FORMAT_32BIT_PLANAR:
		return paInt32;

	case AUDIO_FORMAT_FLOAT:
	case AUDIO_FORMAT_FLOAT_PLANAR:
	default:
		return paFloat32;
	}
	// default to 32 float samples for best quality

}

enum speaker_layout asio_channels_to_obs_speakers(unsigned int channels)
{
	switch (channels) {
	case 1:   return SPEAKERS_MONO;
	case 2:   return SPEAKERS_STEREO;
	case 3:   return SPEAKERS_2POINT1;
	case 4:   return SPEAKERS_4POINT0;
	case 5:   return SPEAKERS_4POINT1;
	case 6:   return SPEAKERS_5POINT1;
		/* no layout for 7 channels */
	case 8:   return SPEAKERS_7POINT1;
	}
	return SPEAKERS_UNKNOWN;
}

/*****************************************************************************/
// get number of output channels
int get_obs_output_channels() {
	// get channel number from output speaker layout set by obs
	struct obs_audio_info aoi;
	obs_get_audio_info(&aoi);
	int recorded_channels = get_audio_channels(aoi.speakers);
	return recorded_channels;
}

// get device count
size_t getDeviceCount() {
	int numDevices;
	numDevices = Pa_GetDeviceCount();
	if (numDevices < 0)
	{
		blog(LOG_ERROR, "ERROR: Pa_CountDevices returned 0x%x\n", numDevices);
	}
	return numDevices;
}

// get the device index
size_t get_device_index(const char *device) {
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	size_t device_index = 0;
	size_t numOfDevices = getDeviceCount();
	for (uint8_t i = 0; i<numOfDevices; i++) {
		deviceInfo = Pa_GetDeviceInfo(i);
		if (strcmp(device, deviceInfo->name) == 0) {
			device_index = i;
			break;
		}
	}
	return device_index;
}
///////////////////////////////
void asio_update(void *vptr, obs_data_t *settings);
void asio_destroy(void *vptr);
///////////////////////////////

// call the control panel
static bool DeviceControlPanel(obs_properties_t *props, 
	obs_property_t *property, void *data) {
	PaError err;
	asio_data *asiodata = (asio_data *)data;

	HWND asio_main_hwnd = (HWND)obs_frontend_get_main_window_handle();
	// stop the stream
	if (Pa_IsStreamActive(asiodata->stream)) {
		err = Pa_Terminate();
		if (err != paNoError) {
			blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
		}
		err = Pa_Initialize();
		if (err != paNoError) {
			blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
		}
	}
	err = PaAsio_ShowControlPanel(asiodata->device_index, asio_main_hwnd);

	if (err == paNoError) {
		blog(LOG_INFO, "Console loaded for device %s with index %i\n",
			asiodata->device, asiodata->device_index);
	}
	else {
		blog(LOG_ERROR, "Could not load the Console panel; PortAudio error : %s\n", Pa_GetErrorText(err));
	}
	// update round
	asio_update((void *)asiodata, asiodata->settings);

	return true;
}

/*****************************************************************************/

//creates the device list
void fill_out_devices(obs_property_t *list) {

	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	int numOfDevices = getDeviceCount();
	blog(LOG_INFO, "ASIO Devices: %i\n", numOfDevices);
	// Scan through devices for various capabilities
	for (int i = 0; i<numOfDevices; i++) {
		deviceInfo = Pa_GetDeviceInfo(i);
		if (deviceInfo) {
			blog(LOG_INFO, "device  %i = %s\n", i, deviceInfo->name);
			blog(LOG_INFO, ": maximum input channels = %i\n", deviceInfo->maxInputChannels);
			blog(LOG_INFO, ": maximum output channels = %i\n", deviceInfo->maxInputChannels);
			blog(LOG_INFO, "list ASIO Devices: %i\n", numOfDevices);
			blog(LOG_INFO, "device %i  = %s added successfully.\n", i, deviceInfo->name);
			obs_property_list_add_string(list, deviceInfo->name, deviceInfo->name);
		}
		else {
			blog(LOG_INFO, "device %i  = %s could not be added: driver issue.\n", i, deviceInfo->name);
		}
	}
}

/* Creates list of input channels.
 * A muted channel has value -1 and is recorded.
 * The user can unmute the channel later.
 */
static bool fill_out_channels_modified(obs_properties_t *props, obs_property_t *list, obs_data_t *settings) {
	const char* device = obs_data_get_string(settings, "device_id");
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	size_t input_channels;
	int index = get_device_index(device);
	const char *channelName = new char;

	//get the device info
	deviceInfo = Pa_GetDeviceInfo(index);
	input_channels = deviceInfo->maxInputChannels;
	
	obs_property_list_clear(list);
	obs_property_list_add_int(list, "mute", -1);
	for (unsigned int i = 0; i < input_channels; i++) {
		std::string namestr = deviceInfo->name;
		namestr += " " + std::to_string(i);
		PaAsio_GetInputChannelName(index, i, &channelName);
		std::string chnamestr = channelName;
		namestr += " " + chnamestr;
		obs_property_list_add_int(list, namestr.c_str(), i);
	}
	return true;
}

// utility function checking if sample rate is supported by device
bool canSamplerate(int device_index, int sample_rate) {
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	deviceInfo = Pa_GetDeviceInfo(device_index);
	PaStreamParameters outputParameters;
	PaStreamParameters inputParameters;
	PaError err;

	memset(&inputParameters, 0, sizeof(inputParameters));
	memset(&outputParameters, 0, sizeof(outputParameters));
	inputParameters.channelCount = deviceInfo->maxInputChannels;
	inputParameters.device = device_index;
	inputParameters.hostApiSpecificStreamInfo = NULL;
	inputParameters.sampleFormat = paFloat32;
	inputParameters.suggestedLatency = deviceInfo->defaultLowInputLatency;
	inputParameters.hostApiSpecificStreamInfo = NULL; 

	outputParameters.channelCount = deviceInfo->maxOutputChannels;
	outputParameters.device = device_index;
	outputParameters.hostApiSpecificStreamInfo = NULL;
	outputParameters.sampleFormat = paFloat32;
	outputParameters.suggestedLatency = deviceInfo->defaultLowOutputLatency;
	outputParameters.hostApiSpecificStreamInfo = NULL; 

	err = Pa_IsFormatSupported(&inputParameters, &outputParameters, sample_rate);
	return (err == paFormatIsSupported) ? true : false;

}

//creates list of input sample rates supported by the device
static bool fill_out_sample_rates(obs_properties_t *props, obs_property_t *list, obs_data_t *settings) {
	const char* device = obs_data_get_string(settings, "device_id");
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	bool ret;
	std::string rate;

	obs_property_list_clear(list);
	int index = get_device_index(device);

	//get the device info
	deviceInfo = Pa_GetDeviceInfo(index);

	//ret = canSamplerate(index, 44100);
	//if (ret) {
	//	rate = std::to_string(44100) + " Hz";
	//	obs_property_list_add_int(list, rate.c_str(), 44100);
	//}
	//ret = canSamplerate(index, 48000);
	//if (ret) {
	//	rate = std::to_string(48000) + " Hz";
	//	obs_property_list_add_int(list, rate.c_str(), 48000);
	//}
	// rearoute samplerate is not probed correctly so ...
	rate = std::to_string(44100) + " Hz";
	obs_property_list_add_int(list, rate.c_str(), 44100);
	rate = std::to_string(48000) + " Hz";
	obs_property_list_add_int(list, rate.c_str(), 48000);
	return true;
}

//create list of supported audio sample formats (supported by obs) excluding 8bit
static bool fill_out_bit_depths(obs_properties_t *props, obs_property_t *list, obs_data_t *settings) {
	const char* device = obs_data_get_string(settings, "device_id");

	obs_property_list_clear(list);
	obs_property_list_add_int(list, "16 bit", AUDIO_FORMAT_16BIT);
	obs_property_list_add_int(list, "32 bit", AUDIO_FORMAT_32BIT);
	obs_property_list_add_int(list, "32 bit float (preferred)", AUDIO_FORMAT_FLOAT);

	return true;
}

//create list of device supported buffer sizes
static bool fill_out_buffer_sizes(obs_properties_t *props, obs_property_t *list, obs_data_t *settings) {
	const char* device = obs_data_get_string(settings, "device_id");
	int index = get_device_index(device);
	long *minBuf = new long;
	long *maxBuf = new long;
	long *BufPref = new long;
	long *gran = new long;
	PaError err;

	err = PaAsio_GetAvailableBufferSizes(index, minBuf, maxBuf, BufPref, gran);
	if (err != paNoError) {
		blog(LOG_ERROR, "Could not retrieve Buffer sizes.\n"
				"PortAudio error: %s\n", Pa_GetErrorText(err));
	}
	else {
		blog(LOG_INFO, "minBuf = %i; maxbuf = %i; bufPref = %i ; gran = %i\n", *minBuf, *maxBuf, *BufPref, *gran);
	}

	obs_property_list_clear(list);

	if (*gran == -1) {
		size_t gran_buffer = *minBuf;
		while (gran_buffer <= *maxBuf) {
			int n = snprintf(NULL, 0, "%llu%s", gran_buffer, (gran_buffer == *BufPref ? " (preferred)" : ""));
			if (n <= 0) {
				//problem...continuing on the loop
				gran_buffer *= 2;
				continue;
			}
			char * buf = (char*)malloc((n + 1) * sizeof(char));
			if (!buf) {
				//problem...continuing on the loop
				gran_buffer *= 2;
				continue;
			}
			int c = snprintf(buf, n + 1, "%llu%s", gran_buffer, (gran_buffer == *BufPref ? " (preferred)" : ""));
			buf[n] = '\0';
			obs_property_list_add_int(list, buf, gran_buffer);
			free(buf);
			gran_buffer *= 2;
		}
	}
	else if (*gran == 0) {
		size_t gran_buffer = *minBuf;
		int n = snprintf(NULL, 0, "%llu%s", gran_buffer, (gran_buffer == *BufPref ? " (preferred)" : ""));
		char * buf = (char*)malloc((n + 1) * sizeof(char));
		int c = snprintf(buf, n + 1, "%llu%s", gran_buffer, (gran_buffer == *BufPref ? " (preferred)" : ""));
		buf[n] = '\0';
		obs_property_list_add_int(list, buf, gran_buffer);
	}
	else if (*gran > 0) {
		size_t gran_buffer = *minBuf;
		while (gran_buffer <= *maxBuf) {
			int n = snprintf(NULL, 0, "%llu%s", gran_buffer, (gran_buffer == *BufPref ? " (preferred)" : ""));
			if (n <= 0) {
				//problem...continuing on the loop
				gran_buffer += *gran;
				continue;
			}
			char * buf = (char*)malloc((n + 1) * sizeof(char));
			if (!buf) {
				//problem...continuing on the loop
				gran_buffer += *gran;
				continue;
			}
			int c = snprintf(buf, n + 1, "%llu%s", gran_buffer, (gran_buffer == *BufPref ? " (preferred)" : ""));
			buf[n] = '\0';
			obs_property_list_add_int(list, buf, gran_buffer);
			free(buf);
			gran_buffer += *gran;
		}
	}

	return true;
}
static bool asio_device_changed(obs_properties_t *props,
	obs_property_t *list, obs_data_t *settings)
{
	const char *curDeviceId = obs_data_get_string(settings, "device_id");
	obs_property_t *sample_rate = obs_properties_get(props, "sample rate");
	obs_property_t *bit_depth = obs_properties_get(props, "bit depth");
	obs_property_t *buffer_size = obs_properties_get(props, "buffer");
	// be sure to set device as current one

	size_t itemCount = obs_property_list_item_count(list);
	bool itemFound = false;

	for (size_t i = 0; i < itemCount; i++) {
		const char *DeviceId = obs_property_list_item_string(list, i);
		if (strcmp(DeviceId, curDeviceId) == 0) {
			itemFound = true;
			break;
		}
	}

	if (!itemFound) {
		obs_property_list_insert_string(list, 0, " ", curDeviceId);
		obs_property_list_item_disable(list, 0, true);
	}
	else {
		DWORD device_index = get_device_index(curDeviceId);
		obs_property_list_clear(sample_rate);
		//obs_property_list_insert_int(sample_rate, 0, " ", 0);
		//obs_property_list_item_disable(sample_rate, 0, true);
		obs_property_list_clear(bit_depth);
		//obs_property_list_insert_int(bit_depth, 0, " ", 0);
		//obs_property_list_item_disable(bit_depth, 0, true);

		//fill out based on device's settings
		obs_property_list_clear(buffer_size);
		//obs_property_list_insert_int(buffer_size, 0, " ", 0);
		//obs_property_list_item_disable(buffer_size, 0, true);
		obs_property_set_modified_callback(sample_rate, fill_out_sample_rates);
		obs_property_set_modified_callback(bit_depth, fill_out_bit_depths);
		obs_property_set_modified_callback(buffer_size, fill_out_buffer_sizes);

		//}
	}
	// get channel number from output speaker layout set by obs
	DWORD recorded_channels = get_obs_output_channels();

	obs_property_t *route[MAX_AUDIO_CHANNELS];
	if (itemFound) {
		for (unsigned int i = 0; i < recorded_channels; i++) {
			std::string name = "route " + std::to_string(i);
			route[i] = obs_properties_get(props, name.c_str());
			obs_property_list_clear(route[i]);
			//			obs_data_set_default_int(settings, name.c_str(), -1); // default is muted channels
			obs_property_set_modified_callback(route[i], fill_out_channels_modified);
		}
	}

	return true;
}

// portaudio callback
int create_asio_buffer(const void *inputBuffer, void *outputBuffer, unsigned long framesCount,
	const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
	void *userData) {
	asio_data *data = (asio_data *)userData;

	int route[MAX_AUDIO_CHANNELS];
	int recorded_channels = data->recorded_channels;
	for (short i = 0; i < MAX_AUDIO_CHANNELS; i++) {
		if (i < recorded_channels) {
			route[i] = data->route[i];
		}
		else {
			route[i] = -1; // not necessary, just avoids it being unset
		}	
	}

	//uint8_t *buffer to have exactly one byte for each pointer increment
	uint8_t *inputBuf = (uint8_t *)inputBuffer;
	uint8_t *outputBuf;

	// won't ever reach that part of code unless we 've made some severe coding error
	if (recorded_channels > MAX_AUDIO_CHANNELS) {
		blog(LOG_ERROR, "OBS does not support more than %i channels",
				MAX_AUDIO_CHANNELS);
		return paAbort;
	}
	///////////////////////////////////////////////////////////////////////////
	/* buffer in Bytes =
	* number of frames in buffer x number of channels x bitdepth / 8
	* buffer per channel in Bytes = number of frames in buffer x bitdepth / 8
	*/
	int BitDepthBytes = bytedepth_format(data->BitDepth);//data->BitDepth);
	size_t inputbufSizeBytes = framesCount * BitDepthBytes * data->channels;
	size_t bufSizePerChannelBytes = framesCount * BitDepthBytes;
	size_t nbFrames = framesCount;
	size_t outputbufSizeBytes = bufSizePerChannelBytes * recorded_channels;

	// for interleaved, the size in bytes of a frame is not the same for input
	// which has all the devices channels and for the buffer passed to obs:
	// the latter has a different number of channels = recorded_channels
	size_t outFrameSize = recorded_channels * BitDepthBytes;
	size_t inputFrameSize = data->channels * BitDepthBytes;
	// allocate outputBuf
	outputBuf = (uint8_t *)calloc(outputbufSizeBytes, 1);
	///////////////////////////////////////////////////////////////////////////

	// interleaved frames
	for (short i = 0; i < nbFrames; i++) {
		for (short j = 0; j < recorded_channels; j++) {
			if (route[j] != -1) {
				memcpy(outputBuf + i * outFrameSize + j * BitDepthBytes, inputBuf + i * inputFrameSize + route[j] * BitDepthBytes, BitDepthBytes);
			}
			// no need to silent the mute channels since they're already calloc'ed to zero == silence
		}
	}
	struct obs_source_audio out;
	
	out.data[0] = outputBuf;
	out.format = AUDIO_FORMAT_FLOAT;
	out.speakers = asio_channels_to_obs_speakers(recorded_channels);
	out.samples_per_sec = data->SampleRate;
	out.frames = nbFrames;
	out.timestamp = os_gettime_ns() - ((nbFrames * NSEC_PER_SEC) / data->SampleRate);

	if (!data->first_ts) {
		data->first_ts = out.timestamp;
	}

	if (out.timestamp > data->first_ts && recorded_channels != 0) {
		obs_source_output_audio(data->source, &out);
	}

	return paContinue;
}

bool asio_init(struct asio_data *data)
{
//	PaStream** stream = new PaStream*;
//	PaError err;
//	int i;
//	// get info, useful for debug
//	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
//	int index = get_device_index(data->device);
//	deviceInfo = Pa_GetDeviceInfo(index);
//
//	unsigned int deviceNumber = getDeviceCount();
//	if (deviceNumber < 1) {
//		blog(LOG_ERROR, "\nNo audio devices found!\n");
//	}
//	/* stream parameters */
//	PaStreamParameters *inParam = new PaStreamParameters();
//	inParam->channelCount = data->channels;
//	inParam->device = index;
//	inParam->sampleFormat = obs_to_portaudio_audio_format(data->BitDepth);
//	inParam->suggestedLatency = 0;
//	inParam->hostApiSpecificStreamInfo = NULL;
//	
//	/* Open an audio I/O stream. */
//	err = Pa_OpenStream(stream, inParam, NULL, data->SampleRate,
//		data->BufferSize, paNoFlag, create_asio_buffer,	data); 
//
//	data->stream = stream;
//	if (err != paNoError) {
//		blog(LOG_ERROR,"Could not open the stream \n");
//		goto error;
//	}
//	// start streaming audio
//	err = Pa_StartStream(*stream);
//	if (err != paNoError) {
//		blog(LOG_ERROR, "Could not start the stream \n");
//		goto error;
//	}
//	return 1;
//error:
//	blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
	return 0;
}

static void * asio_create(obs_data_t *settings, obs_source_t *source)
{
	struct asio_data *data = new asio_data;

	data->source = source;
	data->first_ts = 0;
	data->device = NULL;
	data->info = NULL;
	data->stream = NULL;
	data->settings = settings;

	// check that we're accessing only asio devices
	//assert(Pa_GetHostApiInfo(Pa_GetDeviceInfo(Pa_GetDefaultOutputDevice())->hostApi)->type == paASIO);
	PaError err = Pa_Initialize();
	if (err != paNoError) {
		blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));
		return 0;
	}
	else {
		asio_update(data, settings);
	}

	return data;
}

void asio_destroy(void *vptr)
{
	PaError err;
	struct asio_data *data = (asio_data *)vptr;
	if (Pa_IsStreamActive(data->stream) == 1) {
		err = Pa_AbortStream(data->stream);
		if (err != paNoError)
			blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));
	}
	
	err = Pa_CloseStream(data->stream);
	if (err != paNoError)
		blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));
	err = Pa_Terminate();
	if (err != paNoError)
		blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));

	delete data;
}

/* set all settings to asio_data struct */
void asio_update(void *vptr, obs_data_t *settings)
{
	struct asio_data *data = (asio_data *)vptr;
	const char *device;
	unsigned int rate;
	audio_format BitDepth;
	uint16_t BufferSize;
	unsigned int channels;
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	int res, device_index;
	bool reset = false;
	bool resetDevice = false;
	PaError err;
	PaStream** stream = new PaStream*;
	PaStreamParameters *inParam = new PaStreamParameters();
	int i;
	int route[MAX_AUDIO_CHANNELS];

	int nb = Pa_GetHostApiCount();
	
	// get channel number from output speaker layout set by obs
	int recorded_channels = get_obs_output_channels();
	data->recorded_channels = get_obs_output_channels();

	// get device from settings
	device = obs_data_get_string(settings, "device_id");

	if (device == NULL || device[0] == '\0') {
		printf("ASIO plugin: Error, Device not yet set \n");
	}
	else if (data->device == NULL || data->device[0] == '\0') {
		data->device = bstrdup(device);
		reset = true;
	}
	else {
		if (strcmp(device, data->device) != 0) {
			data->device = bstrdup(device);
			resetDevice = true;
		}
	}
	if (device != NULL && device[0] != '\0') {
		device_index = get_device_index(device);
		deviceInfo = Pa_GetDeviceInfo(device_index);	

		for (int i = 0; i < recorded_channels; i++) {
			std::string route_str = "route " + std::to_string(i);
			route[i] = (int)obs_data_get_int(settings, route_str.c_str());
			if (data->route[i] != route[i]) {
				data->route[i] = route[i];
				reset = true;
			}
		}

		rate = (int)obs_data_get_int(settings, "sample rate");
		if (data->SampleRate != (int)rate) {
			data->SampleRate = (int)rate;
			reset = true;
		}

		BitDepth = (audio_format)obs_data_get_int(settings, "bit depth");
		if (data->BitDepth != BitDepth) {
			data->BitDepth = BitDepth;
			reset = true;
		}

		BufferSize = (uint16_t)obs_data_get_int(settings, "buffer");
		if (data->BufferSize != BufferSize) {
			data->BufferSize = BufferSize;
			reset = true;
		}

		data->channels = deviceInfo->maxInputChannels;
		data->output_channels = deviceInfo->maxOutputChannels;
		data->device_index = device_index;

		/* stream parameters */

		inParam->channelCount = data->channels;
		inParam->device = data->device_index;
		inParam->sampleFormat = obs_to_portaudio_audio_format(data->BitDepth);
		inParam->suggestedLatency = 0;
		inParam->hostApiSpecificStreamInfo = NULL;

		/* Open an audio I/O stream. */
		if (resetDevice) {
			// close old stream
			err = Pa_Terminate();
			if (err != paNoError) {
				blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
			}
			err = Pa_Initialize();
			if (err != paNoError) {
				blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
			}
		}
		if (BitDepth != 0 && (rate == 44100 || rate == 48000) && BufferSize != 0) {
			err = Pa_OpenStream(stream, inParam, NULL, data->SampleRate,
				data->BufferSize, paNoFlag, create_asio_buffer, data);
			data->stream = stream; // update to new stream

			if (err == paNoError) {
				blog(LOG_INFO, "ASIO Stream successfully opened.\n");
				err = Pa_StartStream(*stream);
				if (err == paNoError) {
					blog(LOG_INFO, "ASIO Stream successfully started.\n");
				}
				else {
					blog(LOG_ERROR, "Could not start the stream \n");
					blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
				}
			}
			else {
				blog(LOG_ERROR, "Could not open the stream \n");
				blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
			}
		}
	}

}

const char * asio_get_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return obs_module_text("asioInput");
}

void asio_get_defaults(obs_data_t *settings)
{
	obs_data_set_default_int(settings, "sample rate", 48000);
	obs_data_set_default_int(settings, "bit depth", AUDIO_FORMAT_FLOAT);
	int recorded_channels = get_obs_output_channels();
	for (unsigned int i = 0; i < recorded_channels; i++) {
		std::string name = "route " + std::to_string(i);
		obs_data_set_default_int(settings, name.c_str(), -1); // default is muted channels
	}
}

obs_properties_t * asio_get_properties(void *unused)
{
	obs_properties_t *props;
	obs_property_t *devices;
	obs_property_t *rate;
	obs_property_t *bit_depth;
	obs_property_t *buffer_size;
	obs_property_t *console;
	obs_property_t *route[MAX_AUDIO_CHANNELS];
	int pad_digits = (int)floor(log10(abs(MAX_AUDIO_CHANNELS))) + 1;

	UNUSED_PARAMETER(unused);

	props = obs_properties_create();
//	obs_properties_set_flags(props, OBS_PROPERTIES_DEFER_UPDATE);
	devices = obs_properties_add_list(props, "device_id",
			obs_module_text("Device"), OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING);
	obs_property_set_modified_callback(devices, asio_device_changed);
	fill_out_devices(devices);
	std::string dev_descr = "ASIO devices.\n"
			"OBS-Studio supports for now a single ASIO source.\n"
			"But duplication of an ASIO source in different scenes is still possible";
	obs_property_set_long_description(devices, dev_descr.c_str());
	// get channel number from output speaker layout set by obs
	struct obs_audio_info aoi;
	obs_get_audio_info(&aoi);
	unsigned int recorded_channels = get_audio_channels(aoi.speakers);

	std::string route_descr = "For each OBS output channel, pick one\n of the input channels of your ASIO device.\n";
	const char* route_name_format = "route %i";
	char* route_name = new char[strlen(route_name_format) + pad_digits];

	const char* route_obs_format = "Route.%i";
	char* route_obs = new char[strlen(route_obs_format) + pad_digits];
	for (size_t i = 0; i < recorded_channels; i++) {
		sprintf(route_name, route_name_format, i);
		sprintf(route_obs, route_obs_format, i);
		route[i] = obs_properties_add_list(props, route_name, obs_module_text(route_obs),
			OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
		obs_property_set_long_description(route[i], route_descr.c_str());
	}

	free(route_name);
	free(route_obs);

	rate = obs_properties_add_list(props, "sample rate",
			obs_module_text("SampleRate"), OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_INT);
	std::string rate_descr = "Sample rate : number of samples per channel in one second.\n";
	obs_property_set_long_description(rate, rate_descr.c_str());
	
	bit_depth = obs_properties_add_list(props, "bit depth",
			TEXT_BITDEPTH, OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	std::string bit_descr = "Bit depth : size of a sample in bits and format.\n"
			"Float should be preferred.";
	obs_property_set_long_description(bit_depth, bit_descr.c_str());

	buffer_size = obs_properties_add_list(props, "buffer", TEXT_BUFFER_SIZE,
			OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_INT);
	//obs_property_list_add_int(buffer_size, "64", 64);
	//obs_property_list_add_int(buffer_size, "128", 128);
	//obs_property_list_add_int(buffer_size, "256", 256);
	//obs_property_list_add_int(buffer_size, "512", 512);
	//obs_property_list_add_int(buffer_size, "1024", 1024);
	std::string buffer_descr = "Buffer : number of samples in a single frame.\n"
			"A lower value implies lower latency.\n"
			"256 should be OK for most cards.\n"
			"Warning: the real buffer returned by the device may differ";
	obs_property_set_long_description(buffer_size, buffer_descr.c_str());

	console = obs_properties_add_button(props, "console",
		obs_module_text("ASIO driver control panel"), DeviceControlPanel);
	std::string console_descr = "Make sure your settings in the Driver Control Panel\n"
		"for sample rate and buffer are consistent with what you\n"
		"have set in OBS.";
	obs_property_set_long_description(console, console_descr.c_str());

	return props;
}

bool obs_module_load(void)
{
	struct obs_source_info asio_input_capture = {};
	asio_input_capture.id             = "asio_input_capture";
	asio_input_capture.type           = OBS_SOURCE_TYPE_INPUT;
	asio_input_capture.output_flags   = OBS_SOURCE_AUDIO;
	asio_input_capture.create         = asio_create;
	asio_input_capture.destroy        = asio_destroy;
	asio_input_capture.update         = asio_update;
	asio_input_capture.get_defaults   = asio_get_defaults;
	asio_input_capture.get_name       = asio_get_name;
	asio_input_capture.get_properties = asio_get_properties;

	obs_register_source(&asio_input_capture);
	return true;
}