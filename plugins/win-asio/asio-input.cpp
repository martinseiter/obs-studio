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
#include "gui/asioselector.h"
#include "circle-buffer.h"
#include "portaudio.h"
#include "pa_asio.h"
#include <QWidget>
#include <QMainWindow>
#include <QWindow>
#include <QMessageBox>
#include <QString>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("win-asio", "en-US")

#define blog(level, msg, ...) blog(level, "asio-input: " msg, ##__VA_ARGS__)

AsioSelector *asioselector = NULL;

typedef struct PaAsioDeviceInfo
{
	PaDeviceInfo commonDeviceInfo;
	long minBufferSize;
	long maxBufferSize;
	long preferredBufferSize;
	long bufferGranularity;

}
PaAsioDeviceInfo;

struct paasio_data {
	PaAsioDeviceInfo *info;
	PaStream *stream;
	obs_data_t *settings;
};

/* forward declarations */
void asio_update(void *vptr, obs_data_t *settings);

/* ========================================================================== */
/* =========== conversion between portaudio and obs, utility functions ====== */
/* ===========================================================================*/

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

PaSampleFormat string_to_portaudio_audio_format(std::string format)
{
	const char * name = "16 Bit Int";
	if (strcmp(format.c_str(), name) == 0) {
		return paInt16;
	}
	name = "32 Bit Int";
	if (strcmp(format.c_str(), name) == 0) {
		return paInt32;
	}
	name = "32 Bit Float";
	if (strcmp(format.c_str(), name) == 0) {
		return paFloat32;
	}
	// default to 32 float samples for best quality
	blog(LOG_ERROR, "string error for audio format; defaulting to float");
	return paFloat32;
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

	err = Pa_IsFormatSupported(&inputParameters, &outputParameters, (double)sample_rate);
	return (err == paFormatIsSupported) ? true : false;

}

////////////////////////////////////////////////////////////////////////////////
//////////////////////* asioselector functions *////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

char* get_scene_data_path() {
	const char* scene = obs_frontend_get_current_scene_collection();
	const char* config_path =
		os_get_config_path_ptr("obs-studio\\basic");//\\scenes

	long len = strlen(scene) + strlen("\\") + strlen(config_path) + 1;

	char *scene_data_path = (char *)malloc(len);
	sprintf(scene_data_path, "%s\\%s", config_path, scene);

	return scene_data_path;
}

void save_asio_device(AsioSelector *selector) {
	obs_data_t *device_settings = obs_data_create();
	std::vector<uint32_t> active_devices = selector->getActiveDevices();

	std::string device_name = "";

	double sample_rate = 0;
	uint32_t buffer_size = 0;
	std::string data_format = "";

	double default_sample_rate = 0;
	uint32_t default_buffer_size = 0;
	std::string default_data_format = "";

	bool active_device = false;
	bool use_optimal_format = false;
	bool use_device_timing = false;
	bool use_minimal_latency = false;

	size_t numDevices = selector->getNumberOfDevices();

	obs_data_array_t *asio_devices = obs_data_array_create();

	for (size_t i = 0; i < numDevices; i++) {
		obs_data_t * asio_device_item = obs_data_create();

		obs_data_array_t *obs_array = obs_data_array_create();
		std::vector<double> sample_rates = selector->getSampleRatesForDevice(i);
		for (size_t j = 0; j < sample_rates.size(); j++) {
			obs_data_t* item = obs_data_create();
			//obs_data_set_string(item, "value", )
			obs_data_set_double(item, "value", sample_rates[j]);
			obs_data_array_push_back(obs_array, item);
			obs_data_release(item);
		}
		obs_data_set_array(asio_device_item, "sample_rate_list", obs_array);
		obs_data_array_release(obs_array);

		obs_array = obs_data_array_create();
		std::vector<uint64_t> buffer_sizes = selector->getBufferSizesForDevice(i);
		for (size_t j = 0; j < buffer_sizes.size(); j++) {
			obs_data_t* item = obs_data_create();
			obs_data_set_int(item, "value", buffer_sizes[j]);
			obs_data_array_push_back(obs_array, item);
			obs_data_release(item);
		}
		obs_data_set_array(asio_device_item, "buffer_size_list", obs_array);
		obs_data_array_release(obs_array);

		obs_array = obs_data_array_create();
		std::vector<std::string> audio_formats = selector->getAudioFormatsForDevice(i);
		for (size_t j = 0; j < audio_formats.size(); j++) {
			obs_data_t* item = obs_data_create();
			obs_data_set_string(item, "value", audio_formats[j].c_str());
			obs_data_array_push_back(obs_array, item);
			obs_data_release(item);
		}
		obs_data_set_array(asio_device_item, "audio_format_list", obs_array);
		obs_data_array_release(obs_array);

		device_name = asioselector->getDeviceName(i);
		obs_data_set_string(asio_device_item, "device_name", device_name.c_str());

		active_device = selector->getIsActiveDevice(i);

		sample_rate = selector->getSampleRateForDevice(i);
		buffer_size = selector->getBufferSizeForDevice(i);
		data_format = selector->getAudioFormatForDevice(i);

		default_sample_rate = selector->getDefaultSampleRateForDevice(i);
		default_buffer_size = selector->getDefaultBufferSizeForDevice(i);
		default_data_format = selector->getDefaultAudioFormatForDevice(i);

		use_optimal_format = selector->getUseOptimalFormat(i);
		use_device_timing = selector->getUseDeviceTiming(i);
		use_minimal_latency = selector->getUseMinimalLatency(i);

		obs_data_set_bool(asio_device_item, "_device_active", active_device);
		obs_data_set_bool(asio_device_item, "_use_minimal_latency", use_minimal_latency);
		obs_data_set_bool(asio_device_item, "_use_optimal_format", use_optimal_format);
		obs_data_set_bool(asio_device_item, "_use_device_timing", use_device_timing);

		obs_data_set_double(asio_device_item, "default_sample_rate", default_sample_rate);
		obs_data_set_int(asio_device_item, "default_buffer_size", default_buffer_size);
		obs_data_set_string(asio_device_item, "default_audio_format", default_data_format.c_str());

		if (use_minimal_latency) {
			buffer_size = selector->getBufferSizesForDevice(i)[0];
		}

		obs_data_set_double(asio_device_item, "current_sample_rate", sample_rate);
		obs_data_set_int(asio_device_item, "current_buffer_size", buffer_size);
		obs_data_set_string(asio_device_item, "current_audio_format", data_format.c_str());

		obs_data_array_push_back(asio_devices, asio_device_item);
		obs_data_release(asio_device_item);

	}

	obs_data_set_array(device_settings, "settings", asio_devices);

	const char* scene_data_path = get_scene_data_path();
	size_t scene_data_len = strlen(scene_data_path);
	size_t target_len = scene_data_len + strlen("asio_settings") + strlen("_.json") + 1;
	//-%s
	const char* path_format = "%s_asio_settings.json";
	char* file_path = (char *)malloc(target_len);
	sprintf(file_path, path_format, scene_data_path);

	obs_data_save_json_safe(device_settings, file_path, "tmp", "bak");

	//don't memory leak
	free(file_path);

	obs_data_release(device_settings);
	obs_data_array_release(asio_devices);
}

void load_asio_gui(AsioSelector* selector, AsioSelectorData* old_selector_data) {
	const char* scene_data_path = get_scene_data_path();
	size_t scene_data_len = strlen(scene_data_path);
	size_t target_len = scene_data_len + strlen("asio_settings") + strlen("_.json") + 1;
	//-%s
	const char* path_format = "%s_asio_settings.json";
	char* file_path = (char *)malloc(target_len);
	sprintf(file_path, path_format, scene_data_path);

	obs_data_t *device_settings = obs_data_create_from_json_file_safe(file_path, "bak");
	obs_data_array_t *devices = obs_data_get_array(device_settings, "settings");
	/*todo: fill in gui*/
	size_t count = obs_data_array_count(devices);
	size_t gui_count = selector->getNumberOfDevices();
	size_t j = 0;
	size_t last_found_index = 0;
	for (size_t i = 0; i < gui_count; i++) {
		obs_data_t *item;
		for (j = last_found_index; j < count; j++) {
			item = obs_data_array_item(devices, j);
			std::string file_device_name = std::string(obs_data_get_string(item, "device_name"));
			if (file_device_name == selector->getDeviceName(i)) {
				last_found_index = j;
				break;
			}
			obs_data_release(item);
		}
		if (j < count) {

		}
		else {
			continue;
		}

		selector->setSampleRateForDevice(i, (double)obs_data_get_double(item, "current_sample_rate"));
		selector->setBufferSizeForDevice(i, (uint64_t)obs_data_get_int(item, "current_buffer_size"));
		selector->setAudioFormatForDevice(i, std::string(obs_data_get_string(item, "current_audio_format")));

		selector->setDefaultSampleRateForDevice(i, (double)obs_data_get_double(item, "default_sample_rate"));
		selector->setDefaultBufferSizeForDevice(i, (uint64_t)obs_data_get_int(item, "default_buffer_size"));
		selector->setDefaultAudioFormatForDevice(i, std::string(obs_data_get_string(item, "default_audio_format")));

		selector->setIsActiveDevice(i, (bool)obs_data_get_bool(item, "_device_active"));
		selector->setUseMinimalLatency(i, (bool)obs_data_get_bool(item, "_use_minimal_latency"));
		selector->setUseDeviceTiming(i, (bool)obs_data_get_bool(item, "_use_device_timing"));
		selector->setUseOptimalFormat(i, (bool)obs_data_get_bool(item, "_use_optimal_format"));

		obs_data_release(item);
	}

	//don't memory leak
	free(file_path);
	obs_data_release(device_settings);
	obs_data_array_release(devices);

}

void asio_selector_init(obs_properties_t *props, void *data) {
	asio_listener *listener = (asio_listener *)data;
	if (asioselector == NULL) {
		asioselector = new AsioSelector();
		const PaDeviceInfo *deviceInfo = new PaDeviceInfo;
		size_t numOfDevices = getDeviceCount();
		int device_index;
		long minBuf;
		long maxBuf;
		long BufPref;
		long gran;
		bool ret44, ret48;
		std::vector<uint64_t> buffer_sizes;

		// force sample rates and sample bitdepth to those supported by Obs
		std::vector<double> sample_rates = { 44100, 48000 };
		std::vector<std::string> audio_formats = { "16 Bit Int", "32 Bit Int", "32 Bit Float" };
		for (size_t i = 0; i < numOfDevices; i++) {
			deviceInfo = Pa_GetDeviceInfo(i);
			PaError err;
			err = PaAsio_GetAvailableBufferSizes(i, &minBuf, &maxBuf, &BufPref, &gran);
			if (err != paNoError) {
				blog(LOG_ERROR, "Could not retrieve Buffer sizes.\n"
					"PortAudio error: %s\n", Pa_GetErrorText(err));
			}
			else {
				blog(LOG_INFO, "minBuf = %i; maxbuf = %i; bufPref = %i ; gran = %i\n", minBuf, maxBuf, BufPref, gran);
			}

			// some devices don't report well the buffers they support
			// force buffers to be between 64 to 2048 frames in powers of 2
			// should be fine for most if not all of the devices
			gran = -1;
			minBuf = 64;
			maxBuf = 2048;

			if (gran == -1) {
				uint64_t gran_buffer = minBuf;
				while (gran_buffer <= maxBuf) {
					buffer_sizes.push_back(gran_buffer);
					gran_buffer *= 2;
				}
			}
			else if (gran == 0) {
				size_t gran_buffer = minBuf;
				buffer_sizes.push_back(gran_buffer);
			}
			else if (gran > 0) {
				size_t gran_buffer = minBuf;
				while (gran_buffer <= maxBuf) {
					buffer_sizes.push_back(gran_buffer);
					gran_buffer += gran;
				}
			}

			asioselector->addDevice(std::string(deviceInfo->name), sample_rates, buffer_sizes, audio_formats);
		}
		asioselector->setActiveDeviceUnique(true);
		asioselector->setSaveCallback(save_asio_device);
		load_asio_gui(asioselector, NULL);

		if (asioselector->getActiveDevices().size() != 0) {
			obs_property_t *devices = obs_properties_get(props, "device_id");
			device_index = asioselector->getActiveDevices()[0];
			asioselector->setSelectedDevice(device_index);
			deviceInfo = Pa_GetDeviceInfo(device_index);
			obs_property_list_add_string(devices, deviceInfo->name, deviceInfo->name);
		}
	}

}

static bool asio_selector(obs_properties_t *props,
	obs_property_t *property, void *data) {
	QMainWindow* main_window = (QMainWindow*)obs_frontend_get_main_window();
	asio_selector_init(props, data);
	if (data != NULL)
		asioselector->show();

	return true;
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////// main callbacks for asio_get_properties ////////////////
////////////////////////////////////////////////////////////////////////////////

static bool credits(obs_properties_t *props,
	obs_property_t *property, void *data)
{
	QMainWindow* main_window = (QMainWindow*)obs_frontend_get_main_window();
	QMessageBox mybox(main_window);
	QString text = "(c) 2018, license GPL v2 or later:\r\n"
		"v. 1.0.0 \r\n"
		"Andersama <anderson.john.alexander@gmail.com>\r\n"
		"pkv \r\n <pkv.stream@gmail.com>\r\n";
	mybox.setText(text);
	mybox.setIconPixmap(QPixmap(":/res/images/asiologo.png"));
	mybox.setWindowTitle(QString("Credits: obs-asio"));
	mybox.exec();
	return true;
}

// call the control panel
static bool DeviceControlPanel(obs_properties_t *props, 
	obs_property_t *property, void *data) {
	PaError err;
	asio_listener *listener = (asio_listener*)data;
	paasio_data *paasiodata = (paasio_data *)listener->get_user_data();
	//asio_data *asiodata = (asio_data *)data;

	HWND asio_main_hwnd = (HWND)obs_frontend_get_main_window_handle();
	// stop the stream
	err = Pa_IsStreamActive(paasiodata->stream);
	if (err == 1) {
		err = Pa_CloseStream(paasiodata->stream);
		if (err != paNoError) {
			blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));	
		}
		err = Pa_Terminate();
		if (err != paNoError) {
			blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
		}
		err = Pa_Initialize();
		if (err != paNoError) {
			blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
		}
	}
	err = PaAsio_ShowControlPanel(listener->device_index, asio_main_hwnd);

	if (err == paNoError) {
		blog(LOG_INFO, "Console loaded for device %s with index %i\n",
			listener->get_id(), listener->device_index);
	}
	else {
		blog(LOG_ERROR, "Could not load the Console panel; PortAudio error : %s\n", Pa_GetErrorText(err));
	}
	// update round
	asio_update((void *)listener, paasiodata->settings);// asiodata->settings);

	return true;
}

/* Creates list of input channels.
 * A muted channel has value -1 and is recorded.
 * The user can unmute the channel later.
 */
static bool fill_out_channels_modified(obs_properties_t *props, obs_property_t *list, obs_data_t *settings) {
	const char* device = obs_data_get_string(settings, "device_id");
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	size_t input_channels;
	int index =  asioselector->getActiveDevices()[0];/*get_device_index(device)*/;
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

// portaudio callback
int create_asio_buffer(const void *inputBuffer, void *outputBuffer, unsigned long framesCount,
	const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
	void *userData) {
	uint64_t ts = os_gettime_ns();
	device_buffer *device = (device_buffer*)userData;
	size_t buf_size = device->get_input_channels() * framesCount * bytedepth_format(device->get_format());
//	device->write_buffer_interleaved(inputBuffer, buf_size, ts);
	device->write_buffer_planar(inputBuffer, buf_size, ts);
	return paContinue;

}

static void * asio_create(obs_data_t *settings, obs_source_t *source)
{
	//struct asio_data *data = new asio_data;
	asio_listener *data = new asio_listener();
	struct paasio_data *user_data = new paasio_data;

	data->source = source;
	data->first_ts = 0;
	data->device_name = "";
	user_data->settings = settings;
	user_data->info = NULL;
	user_data->stream = NULL;
	data->set_user_data(user_data);

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
	asio_listener *data = (asio_listener *)vptr;
	delete data;
}

/* set all settings to asio_data struct */
void asio_update(void *vptr, obs_data_t *settings)
{
	//struct asio_data *data = (asio_data *)vptr;
	asio_listener *listener = (asio_listener *)vptr;
	paasio_data *user_data = (paasio_data *)listener->get_user_data();
	const char *device;
	unsigned int rate, streamRate;
	audio_format BitDepth;
	uint16_t BufferSize;
	unsigned int channels;
	const PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	PaAsioDeviceInfo *asioInfo = new PaAsioDeviceInfo;
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
	listener->input_channels = recorded_channels;
	//data->recorded_channels = get_obs_output_channels();

	if (asioselector == NULL) {
		obs_properties_t *props;
		props = obs_properties_create();
		obs_property_t *devices;
		devices = obs_properties_add_list(props, "device_id",
			obs_module_text("Device"), OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING);
		asio_selector(props, devices, NULL);
	}	

	// get device from settings
	device = obs_data_get_string(settings, "device_id");

	if (device == NULL || device[0] == '\0') {
		printf("ASIO plugin: Error, Device not yet set \n");
	}
	else {
		//data->device = bstrdup(device);
		listener->device_name = bstrdup(device);
		if (strcmp(device, listener->device_name.c_str()) != 0) {
			resetDevice = true;
		}
	}

	if (device != NULL && device[0] != '\0') {
		device_index = get_device_index(device);
		listener->device_index = device_index;
		deviceInfo = Pa_GetDeviceInfo(device_index);	

		for (int i = 0; i < recorded_channels; i++) {
			std::string route_str = "route " + std::to_string(i);
			route[i] = (int)obs_data_get_int(settings, route_str.c_str());
			if (listener->route[i] != route[i]) {
				listener->route[i] = route[i];
				reset = true;
			}
		}
		/* sample rate from asioselector */
		rate = asioselector->getSampleRateForDevice(device_index);

		/* retrieve sampleFormat from global asioselector */
		std::string data_format = asioselector->getAudioFormatForDevice(device_index);
		PaSampleFormat format_portaudio = string_to_portaudio_audio_format(data_format);
		BitDepth = portaudio_to_obs_audio_format(format_portaudio);

		/* get buffer from asioselector */
		BufferSize = asioselector->getBufferSizeForDevice(device_index);


		listener->muted_chs = listener->_get_muted_chs(listener->route);
		listener->unmuted_chs = listener->_get_unmuted_chs(listener->route);

		listener->input_channels = deviceInfo->maxInputChannels;
		listener->output_channels = deviceInfo->maxOutputChannels;
		listener->device_index = device_index;

		/* stream parameters */

		inParam->channelCount = deviceInfo->maxInputChannels;//data->channels;
		inParam->device = device_index;//data->device_index;
		inParam->sampleFormat = format_portaudio | paNonInterleaved;//obs_to_portaudio_audio_format(data->BitDepth);
		inParam->suggestedLatency = 0;
		inParam->hostApiSpecificStreamInfo = NULL;

		bool canDo44 = false;
		if (Pa_IsFormatSupported(inParam, NULL, 44100) == paFormatIsSupported)
		{
			canDo44 = true;
		}
		bool canDo48 = false;
		if (Pa_IsFormatSupported(inParam, NULL, 48000) == paFormatIsSupported)
		{
			canDo48 = true;
		}
		
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
		if (!canDo44 && !canDo48) {
			blog(LOG_ERROR, "Device error: not supporting 44100 or 48000 Hz sample rate\n"
				"or not set at either rates.\n"
				"Obs only supports 44100 and 48000 Hz.\n");
		} else if (BitDepth != 0  && BufferSize != 0) {
			if (rate == 44100 && canDo44 || rate == 48000 && canDo48) {
				streamRate = rate;
			}
			else if (rate == 44100 && !canDo44 && canDo48) {
				streamRate = 48000;
			}
			else if (rate == 48000 && !canDo48 && canDo44) {
				streamRate = 44100;
			}

			device_buffer * devicebuf = device_list[device_index];
			listener->disconnect();
			if (devicebuf->get_listener_count() > 0) {
				
			}
			else {
				long min_buf;
				long max_buf;
				long pref_buf;
				long gran;
				PaAsio_GetAvailableBufferSizes(device_index, &min_buf, &max_buf, &pref_buf, &gran);
				devicebuf->prep_circle_buffer(BufferSize);
				devicebuf->prep_events(deviceInfo->maxInputChannels);
				devicebuf->prep_buffers(BufferSize, deviceInfo->maxInputChannels, BitDepth, streamRate);

				err = Pa_OpenStream(stream, inParam, NULL, streamRate,
					BufferSize, paClipOff, create_asio_buffer, devicebuf);
				if (err != paNoError)
					blog(LOG_INFO, "Device is already opened in another asio source!\n");
				user_data->stream = *stream; // update to new stream
				user_data->settings = settings;
				asioInfo->commonDeviceInfo = *deviceInfo;
				asioInfo->minBufferSize = min_buf;
				asioInfo->maxBufferSize = max_buf;
				asioInfo->preferredBufferSize = pref_buf;
				asioInfo->bufferGranularity = gran;
				user_data->info = asioInfo;
				listener->set_user_data(user_data);

				if (err == paNoError) {
					blog(LOG_INFO, "ASIO Stream successfully opened.\n");
					err = Pa_StartStream(*stream);
					if (err == paNoError) {
						blog(LOG_INFO, "ASIO Stream successfully started.\n");
					}
					else {
						blog(LOG_ERROR, "Could not start the stream \n");
						blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
						// normally we should never reach that but some drivers can be buggy
						if (err == paInvalidSampleRate) {
							if (rate == 44100 && canDo48) {
								err = Pa_OpenStream(stream, inParam, NULL, 48000,
									BufferSize, paClipOff, create_asio_buffer, devicebuf);
							}
							else if (rate == 48000 && canDo44) {
								err = Pa_OpenStream(stream, inParam, NULL, 44100,
									BufferSize, paClipOff, create_asio_buffer, devicebuf);
							}

						}
					}
				}
				else {
					blog(LOG_ERROR, "Could not open the stream \n");
					blog(LOG_ERROR, "PortAudio error : %s\n", Pa_GetErrorText(err));
				}

			}
			devicebuf->add_listener(listener);
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
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	int device_index;
	int input_channels;

	UNUSED_PARAMETER(unused);

	props = obs_properties_create();

	devices = obs_properties_add_list(props, "device_id",
			obs_module_text("Device"), OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING);
	asio_selector(props, devices, NULL);
	if (asioselector->getActiveDevices().size() != 0) {
		device_index = asioselector->getActiveDevices()[0];
		deviceInfo = Pa_GetDeviceInfo(device_index);
		obs_property_list_add_string(devices, deviceInfo->name, deviceInfo->name);
	}
	else {
		obs_property_list_insert_string(devices, 0, " ", 0);
		obs_property_list_item_disable(devices, 0, true);
	}

	std::string dev_descr = "ASIO devices.\n"
			"OBS-Studio supports for now a single ASIO source.\n"
			"But duplication of an ASIO source in different scenes is still possible";
	obs_property_set_long_description(devices, dev_descr.c_str());

	obs_property_t *gui = obs_properties_add_button(props, "asio gui", "Change device", asio_selector);
	std::string sel_descr = "ASIO devices.\n"
		"Change ASIO device.\n"
		"Change its settings.";
	obs_property_set_long_description(gui, sel_descr.c_str());

	unsigned int recorded_channels = get_obs_output_channels();

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
		std::string name = "route " + std::to_string(i);
		if (asioselector->getActiveDevices().size() != 0) {
			obs_property_set_modified_callback(route[i], fill_out_channels_modified);
		}
	}

	free(route_name);
	free(route_obs);

	console = obs_properties_add_button(props, "console",
		obs_module_text("ASIO driver control panel"), DeviceControlPanel);
	std::string console_descr = "Make sure your settings in the Driver Control Panel\n"
		"for sample rate and buffer are consistent with what you\n"
		"have set in OBS.";
	obs_property_set_long_description(console, console_descr.c_str());
	obs_property_t *button = obs_properties_add_button(props, "credits", "CREDITS", credits);

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

	PaError err = Pa_Initialize();

	const PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	size_t numOfDevices = getDeviceCount();
	blog(LOG_INFO, "ASIO Devices: %i\n", numOfDevices);
	device_list.reserve(numOfDevices);
	// Scan through devices for various capabilities
	for (int i = 0; i<numOfDevices; i++) {
		deviceInfo = Pa_GetDeviceInfo(i);
		if (deviceInfo) {
			blog(LOG_INFO, "device %i = %s\n", i, deviceInfo->name);
			blog(LOG_INFO, ": maximum input channels = %i\n", deviceInfo->maxInputChannels);
			blog(LOG_INFO, ": maximum output channels = %i\n", deviceInfo->maxInputChannels);
			blog(LOG_INFO, "list ASIO Devices: %i\n", numOfDevices);
			blog(LOG_INFO, "device %i  = %s added successfully.\n", i, deviceInfo->name);
		}
		else {
			blog(LOG_INFO, "device %i = %s could not be added: driver issue.\n", i, deviceInfo->name);
		}
		device_buffer *device = new device_buffer();
		device->device_index = i;
		device->device_options.name = bstrdup(deviceInfo->name);
		device->device_options.channel_count = deviceInfo->maxInputChannels;
		device_list.push_back(device);
	}

	obs_register_source(&asio_input_capture);
	return true;
}