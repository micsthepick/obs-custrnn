cmake_minimum_required(VERSION 3.16...3.21)

project(obs-custrnn VERSION 1.0.0)
add_library(${CMAKE_PROJECT_NAME} MODULE)

set(PLUGIN_AUTHOR "micsthepick")

# Replace 'https://www.example.com` with a link to the website of your plugin or repository
set(PLUGIN_WEBSITE "https://www.github.com/micsthepick/obs-custrnn")

set(MACOS_BUNDLEID "com.example.${CMAKE_PROJECT_NAME}")

set(LINUX_MAINTAINER_EMAIL "micksthepick.bots@gmail.com")

# Add your custom source files here - header files are optional and only required for visibility
# e.g. in Xcode or Visual Studio
target_sources(${CMAKE_PROJECT_NAME} PRIVATE src/plugin-main.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE src/noise_supress_filter_cust.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/include)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/denoise.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/celt_lpc.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/kiss_fft.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/pitch.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/rnn.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/rnn_data.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/rnnoise/src/rnn_reader.c)
add_compile_definitions("COMPILE_OPUS")

# Import libobs as main plugin dependency
find_package(libobs REQUIRED)
include(cmake/ObsPluginHelpers.cmake)

# Uncomment these lines if you want to use the OBS Frontend API in your plugin
#[[
find_package(obs-frontend-api REQUIRED)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE OBS::obs-frontend-api)
#]]

# Uncomment those lines if you want to use Qt in your plugin
#[[
find_qt(COMPONENTS Widgets Core)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Qt::Core Qt::Widgets)
set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES AUTOMOC ON
             AUTOUIC ON
             AUTORCC ON)
#]]

configure_file(src/plugin-macros.h.in ${CMAKE_SOURCE_DIR}/src/plugin-macros.generated.h)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE src/plugin-macros.generated.h)

# /!\ TAKE NOTE: No need to edit things past this point /!\

# --- Platform-independent build settings ---

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE OBS::libobs)

# --- End of section ---

# --- Windows-specific build settings and tasks ---
if(OS_WINDOWS)
  configure_file(cmake/bundle/windows/installer-Windows.iss.in
                 ${CMAKE_BINARY_DIR}/installer-Windows.generated.iss)

  configure_file(cmake/bundle/windows/resource.rc.in ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.rc)
  target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.rc)

  if(MSVC)
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE /W4 /wd4244 /wd4456)
  endif()
  # --- End of section ---

  # -- macOS specific build settings and tasks --
elseif(OS_MACOS)
  configure_file(cmake/bundle/macos/installer-macos.pkgproj.in
                 ${CMAKE_BINARY_DIR}/installer-macos.generated.pkgproj)

  set(MACOSX_PLUGIN_GUI_IDENTIFIER "${MACOS_BUNDLEID}")
  set(MACOSX_PLUGIN_BUNDLE_VERSION "${CMAKE_PROJECT_VERSION}")
  set(MACOSX_PLUGIN_SHORT_VERSION_STRING "1")

  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall -Wno-null-dereference)
  # --- End of section ---

  # --- Linux-specific build settings and tasks ---
else()
  target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wall)
endif()
# --- End of section ---

setup_plugin_target(${CMAKE_PROJECT_NAME})
