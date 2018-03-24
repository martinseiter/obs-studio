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

#define NSEC_PER_SEC  1000000000LL

#define TEXT_BUFFER_SIZE                obs_module_text("BufferSize")
#define TEXT_BUFFER_64_SAMPLES          obs_module_text("64_samples")
#define TEXT_BUFFER_128_SAMPLES         obs_module_text("128_samples")
#define TEXT_BUFFER_256_SAMPLES         obs_module_text("256_samples")
#define TEXT_BUFFER_512_SAMPLES         obs_module_text("512_samples")
#define TEXT_BUFFER_1024_SAMPLES        obs_module_text("1024_samples")
#define TEXT_BITDEPTH                   obs_module_text("BitDepth")

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
/*
struct asio_data {
	obs_source_t *source;

	//asio device and info
	const char *device;
	uint8_t device_index;
	PaAsioDeviceInfo *info;
	PaStream *stream;
	obs_data_t *settings;

	audio_format BitDepth; // 16bit or 32 bit
	int SampleRate;          //44100 or 48000 Hz
	uint16_t BufferSize;     // number of samples in buffer
	uint64_t first_ts;       //first timestamp

	//channels info
	int channels; //total number of input channels
	int output_channels; // number of output channels of device (not used)
	int recorded_channels; // number of channels passed from device (including muted) to OBS; is at most 8
	int route[MAX_AUDIO_CHANNELS]; // stores the channel re-ordering info
};
*/

void activate_asio_device();
void load_asio_device();

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
	//obs_data_set_string(active_device_settings, "", "");
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

	size_t n = selector->getNumberOfDevices();

	obs_data_array_t *asio_devices = obs_data_array_create();
	
	for (size_t i = 0; i < n; i++) {
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

		//obs_data_set_array(current_gui_settings, "_device_active", );
		//obs_data_set_array(current_gui_settings, "_use_minimal_latency", );
		//obs_data_set_array(current_gui_settings, "_use_optimal_format", );
		//obs_data_set_array(current_gui_settings, "_use_device_timing", );

		if (use_optimal_format) {
			//Todo: change between optimal formats
			// 16 bit integer 
			// 32 bit integer
			// 32 bit floating point (obs native) <-
			//pick device native format but prefer floating point if possible
		}
		else {

		}
		if (use_device_timing) {

		}
		if (use_minimal_latency) {
			buffer_size = selector->getBufferSizesForDevice(i)[0];
		}
		else {
			
		}

		obs_data_set_double(asio_device_item, "current_sample_rate", sample_rate);
		obs_data_set_int(asio_device_item, "current_buffer_size", buffer_size);
		obs_data_set_string(asio_device_item, "current_audio_format", data_format.c_str());

		obs_data_array_push_back(asio_devices, asio_device_item);
		obs_data_release(asio_device_item);

		/*
		size_t device_len = device_name.length();
		const char* scene_data_path = get_scene_data_path();
		size_t scene_data_len = strlen(scene_data_path);
		size_t target_len = scene_data_len + device_len + strlen("-.json") + 1;

		const char* path_format = "%s-%s.json";
		char* file_path = (char *)malloc(target_len);
		sprintf(file_path, path_format, scene_data_path, device_name);

		obs_data_save_json_safe(active_device_settings, file_path, "tmp", "bak");
		
		if (active_device) {
			device_name = "active_device_" + std::to_string(i);
			device_len = device_name.length();
			target_len = scene_data_len + device_len + strlen("-.json") + 1;
			if (file_path) {
				free(file_path);
			}
			file_path = (char *)malloc(target_len);
			sprintf(file_path, path_format, scene_data_path, device_name);

			obs_data_save_json_safe(active_device_settings, file_path, "tmp", "bak");

			active_devices.erase(active_devices.begin());
		}
		
		//don't memory leak
		free(file_path);
		*/
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

	activate_asio_device();
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

void activate_asio_device() {
	double sample_rate = 0;
	uint32_t buffer_size = 0;
	std::string data_format = "";
	bool active_device = false;
	bool use_optimal_format = false;
	bool use_device_timing = false;
	bool use_minimal_latency = false;
	const PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	PaStreamParameters *inParam = new PaStreamParameters();

	std::vector<uint32_t> active_devices = asioselector->getActiveDevices();
	int device_index = active_devices[0];
	deviceInfo = Pa_GetDeviceInfo(device_index);

	if (active_devices.size() > 0) {
		sample_rate = asioselector->getSampleRateForDevice(active_devices[0]);
		buffer_size = asioselector->getBufferSizeForDevice(active_devices[0]);
		data_format = asioselector->getAudioFormatForDevice(active_devices[0]);
		active_device = true;
		use_optimal_format = asioselector->getUseOptimalFormat(active_devices[0]);
		use_device_timing = asioselector->getUseDeviceTiming(active_devices[0]);
		use_minimal_latency = asioselector->getUseMinimalLatency(active_devices[0]);
	}
	/* stream parameters */

	inParam->channelCount = deviceInfo->maxInputChannels;//data->channels;
	inParam->device = device_index;//data->device_index;
	inParam->sampleFormat = string_to_portaudio_audio_format(data_format);
	inParam->suggestedLatency = 0;
	inParam->hostApiSpecificStreamInfo = NULL;

	/*
	if (use_optimal_format) {
		//Todo: change between optimal formats
		// 16 bit integer 
		// 32 bit integer
		// 32 bit floating point (obs native) <-
		//pick device native format but prefer floating point if possible
		obs_data_set_string(active_device_settings, "data_format", data_format.c_str());
	}
	else {
		obs_data_set_string(active_device_settings, "data_format", data_format.c_str());
	}
	*/
	if (use_device_timing) {

	}
	if (use_minimal_latency) {
		buffer_size = asioselector->getBufferSizesForDevice(active_devices[0])[0];
	}
	//switch devices / activate devices
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
		for (j = last_found_index+1; j < count; j++) {
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

		selector->setSampleRateForDevice(i, (double)obs_data_get_double(item,"current_sample_rate"));
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

	//store settings in gui
	activate_asio_device();
}

void load_asio_device() {
	//reading json file
	const char* scene_data_path = get_scene_data_path();
	size_t scene_data_len = strlen(scene_data_path);
	size_t target_len = scene_data_len + strlen("asio_settings") + strlen("_.json") + 1;
	//-%s
	const char* path_format = "%s_asio_settings.json";
	char* file_path = (char *)malloc(target_len);
	sprintf(file_path, path_format, scene_data_path);

	obs_data_t *device_settings = obs_data_create_from_json_file_safe(file_path, "bak");
	obs_data_array_t *devices = obs_data_get_array(device_settings, "settings");

	//don't memory leak

	free(file_path);

	//store settings in gui
	activate_asio_device();
}

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
static bool credits(obs_properties_t *props,
	obs_property_t *property, void *data)
{
	QMainWindow* main_window = (QMainWindow*)obs_frontend_get_main_window();
	QMessageBox mybox(main_window);
//	mybox->icon(QMessageBox::Information);
	QString text = "(c) 2018, license GPL v2 or later:\r\n"
		"Andersama <anderson.john.alexander@gmail.com>\r\n"
		"pkv \r\n <pkv.stream@gmail.com>\r\n";
	mybox.setText(text);
	mybox.setIconPixmap(QPixmap(":/res/images/asiologo.png"));
	mybox.setWindowTitle(QString("Credits: obs-asio"));
	mybox.exec();
	return true;
}

void asio_selector_init() {
	if (asioselector == NULL) {
		asioselector = new AsioSelector();
		const PaDeviceInfo *deviceInfo = new PaDeviceInfo;
		size_t numOfDevices = getDeviceCount();
		long minBuf;
		long maxBuf;
		long BufPref;
		long gran;
		std::vector<double> sample_rates = { 44100, 48000 };
		std::vector<std::string> audio_formats = { "16 Bit Int", "32 Bit Int", "32 Bit Float" };
		for (size_t i = 0; i < numOfDevices; i++) {
			deviceInfo = Pa_GetDeviceInfo(i);
			PaError err;
			err = PaAsio_GetAvailableBufferSizes(i, &minBuf, &maxBuf, &BufPref, &gran);
			std::vector<uint64_t> buffer_sizes;

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

	}
}

static bool asio_selector(obs_properties_t *props,
	obs_property_t *property, void *data) {
	QMainWindow* main_window = (QMainWindow*)obs_frontend_get_main_window();
	//QDialog dialog(main_window);
	//static AsioSelector w;
	asio_selector_init();
	asioselector->show();
	
	return true;
	//return dialog.exec();
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

	err = Pa_IsFormatSupported(&inputParameters, &outputParameters, (double)sample_rate);
	return (err == paFormatIsSupported) ? true : false;

}

//creates list of input sample rates supported by the device
static bool fill_out_sample_rates(obs_properties_t *props, obs_property_t *list, obs_data_t *settings) {
	const char* device = obs_data_get_string(settings, "device_id");
	const   PaDeviceInfo *deviceInfo = new PaDeviceInfo;
	bool ret44, ret48;
	std::string rate;

	obs_property_list_clear(list);
	int index = get_device_index(device);

	//get the device info
	deviceInfo = Pa_GetDeviceInfo(index);

	ret44 = canSamplerate(index, 44100);
	if (ret44) {
		rate = std::to_string(44100) + " Hz";
		obs_property_list_add_int(list, rate.c_str(), 44100);
	}
	ret48 = canSamplerate(index, 48000);
	if (ret48) {
		rate = std::to_string(48000) + " Hz";
		obs_property_list_add_int(list, rate.c_str(), 48000);
	}
	if (!ret44 && !ret48) {
		blog(LOG_ERROR, "Obs only supports 44100 and 48000 Hz.\n"
				"Change the smaple rate of your device.\n");
	}
	// rearoute samplerate is not probed correctly so ...
	/*rate = std::to_string(44100) + " Hz";
	obs_property_list_add_int(list, rate.c_str(), 44100);
	rate = std::to_string(48000) + " Hz";
	obs_property_list_add_int(list, rate.c_str(), 48000);*/
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
		long long gran_buffer = *minBuf;
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
	//(uint64_t)(timeInfo->inputBufferAdcTime * NSEC_PER_SEC)
	//timeInfo->currentTime
	//timeInfo->inputBufferAdcTime
	uint64_t ts = os_gettime_ns();
	device_buffer *device = (device_buffer*)userData;
	size_t buf_size = device->get_input_channels() * framesCount * bytedepth_format(device->get_format());
	device->write_buffer_interleaved(inputBuffer, buf_size, ts);

	return paContinue;
	/*
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
	// buffer in Bytes =
	// number of frames in buffer x number of channels x bitdepth / 8
	// buffer per channel in Bytes = number of frames in buffer x bitdepth / 8

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
	*/
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
	//struct asio_data *data = new asio_data;
	asio_listener *data = new asio_listener();
	struct paasio_data *user_data = new paasio_data;

	data->source = source;
	data->first_ts = 0;
	data->device_name = "";
	//data->device = NULL;
	//data->info = NULL;
	//data->stream = NULL;
	//data->settings = settings;
	user_data->settings = settings;
	user_data->info = NULL;
	user_data->stream = NULL;
	data->set_user_data(user_data);

//	load_asio_gui(asioselector, NULL);

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
	//PaError err;
	//struct asio_data *data = (asio_data *)vptr;
	asio_listener *data = (asio_listener *)vptr;
	/*
	paasio_data *user_data = (paasio_data *)data->get_user_data();
	if (Pa_IsStreamActive(user_data->stream)){//data->stream) == 1) {
		err = Pa_AbortStream(user_data->stream);
		if (err != paNoError)
			blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));
	}
	
	err = Pa_CloseStream(data->stream);
	if (err != paNoError)
		blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));
	err = Pa_Terminate();
	if (err != paNoError)
		blog(LOG_ERROR, "PortAudio error: %s\n", Pa_GetErrorText(err));
	*/
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

	// get device from settings
	device = obs_data_get_string(settings, "device_id");

	if (device == NULL || device[0] == '\0') {
		printf("ASIO plugin: Error, Device not yet set \n");
	}
	/*
	else if(listner->device_name == "")  {
	//else if (data->device == NULL || data->device[0] == '\0') {
			
	//data->device = bstrdup(device);
		reset = true;
	}
	*/
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

		rate = (int)obs_data_get_int(settings, "sample rate");
		/*
		if (data->SampleRate != (int)rate) {
			data->SampleRate = (int)rate;

			reset = true;
		}
		*/

		BitDepth = (audio_format)obs_data_get_int(settings, "bit depth");
		/*
		if (data->BitDepth != BitDepth) {
			data->BitDepth = BitDepth;
			reset = true;
		}
		*/

		BufferSize = (uint16_t)obs_data_get_int(settings, "buffer");
		/*
		if (data->BufferSize != BufferSize) {
			data->BufferSize = BufferSize;
			reset = true;
		}
		*/

		listener->muted_chs = listener->_get_muted_chs(listener->route);
		listener->unmuted_chs = listener->_get_unmuted_chs(listener->route);

		listener->input_channels = deviceInfo->maxInputChannels;
		listener->output_channels = deviceInfo->maxOutputChannels;
		listener->device_index = device_index;
		//data->channels = deviceInfo->maxInputChannels;
		//data->output_channels = deviceInfo->maxOutputChannels;
		//data->device_index = device_index;

		/* stream parameters */

		inParam->channelCount = deviceInfo->maxInputChannels;//data->channels;
		inParam->device = device_index;//data->device_index;
		inParam->sampleFormat = obs_to_portaudio_audio_format(BitDepth);//obs_to_portaudio_audio_format(data->BitDepth);
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
				streamRate = rate;//data->SampleRate;
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
				//user_data->info;
				//user_data->settings;
				//user_data->stream;
				devicebuf->prep_circle_buffer(pref_buf);
				devicebuf->prep_events(deviceInfo->maxInputChannels);
				devicebuf->prep_buffers(pref_buf, deviceInfo->maxInputChannels, BitDepth, streamRate);

				err = Pa_OpenStream(stream, inParam, NULL, streamRate,
					pref_buf, paClipOff, create_asio_buffer, devicebuf);
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
									pref_buf, paClipOff, create_asio_buffer, devicebuf);
							}
							else if (rate == 48000 && canDo44) {
								err = Pa_OpenStream(stream, inParam, NULL, 44100,
									pref_buf, paClipOff, create_asio_buffer, devicebuf);
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
	
	devices = obs_properties_add_list(props, "device_id",
			obs_module_text("Device"), OBS_COMBO_TYPE_LIST,
			OBS_COMBO_FORMAT_STRING);
	obs_property_set_modified_callback(devices, asio_device_changed);
	fill_out_devices(devices);
	std::string dev_descr = "ASIO devices.\n"
			"OBS-Studio supports for now a single ASIO source.\n"
			"But duplication of an ASIO source in different scenes is still possible";
	obs_property_set_long_description(devices, dev_descr.c_str());

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
	obs_property_t *button = obs_properties_add_button(props, "credits", "CREDITS", credits);
	
	obs_property_t *gui = obs_properties_add_button(props, "asio gui", "main device", asio_selector);

	//QLabel test;
	//test.setText(QString("test"));
	//QMainWindow* main_window = (QMainWindow*)obs_frontend_get_main_window();
	//test.setParent(main_window);
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
			//obs_property_list_add_string(list, deviceInfo->name, deviceInfo->name);
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