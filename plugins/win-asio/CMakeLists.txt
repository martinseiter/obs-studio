project(win-asio)

find_package(LibObs REQUIRED)

include_directories(
	SYSTEM "${CMAKE_SOURCE_DIR}/libobs"
	"${CMAKE_SOURCE_DIR}/UI/obs-frontend-api"
	${PORTAUDIO_INCLUDE_DIR}
)
set(win-asio-sdk_HEADERS
	)

set(win-asio_HEADERS
	)

set(win-asio_SOURCES
	asio-input.cpp
)

add_library(win-asio MODULE
	${win-asio_SOURCES}
	${win-asio_HEADERS}
	${win-asio-sdk_HEADERS}
)
target_link_libraries(win-asio
	libobs
	obs-frontend-api
	${PORTAUDIO_LIBRARY}
)

install_obs_plugin_with_data(win-asio data)
