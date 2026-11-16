# Official SDK C ABI: install the release separately, never download at build time.
set(WUJI_SDK_VERSION "2026.10.26")
set(WUJI_SDK_ROOT "" CACHE PATH "Extracted Wuji SDK C 2026.10.26 release")
find_path(WUJI_SDK_INCLUDE_DIR wuji_sdk.h HINTS "${WUJI_SDK_ROOT}/include"
  REQUIRED)
find_library(WUJI_SDK_LIBRARY wuji_sdk_c HINTS "${WUJI_SDK_ROOT}/lib" REQUIRED)
get_filename_component(WUJI_SDK_LIBRARY_DIR "${WUJI_SDK_LIBRARY}" DIRECTORY)
add_library(WujiSdk::C SHARED IMPORTED)
set_target_properties(WujiSdk::C PROPERTIES
  IMPORTED_LOCATION "${WUJI_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${WUJI_SDK_INCLUDE_DIR}")

# wuji_version has no device side effects; verify the actual linked release.
try_run(WUJI_SDK_VERSION_RESULT WUJI_SDK_VERSION_COMPILED
  "${CMAKE_CURRENT_BINARY_DIR}/sdk_version"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_sdk_version.cpp"
  CMAKE_FLAGS "-DINCLUDE_DIRECTORIES=${WUJI_SDK_INCLUDE_DIR}"
  LINK_LIBRARIES "${WUJI_SDK_LIBRARY}"
  RUN_OUTPUT_VARIABLE WUJI_SDK_DETECTED_VERSION)
if(NOT WUJI_SDK_VERSION_COMPILED OR NOT WUJI_SDK_VERSION_RESULT EQUAL 0
    OR NOT WUJI_SDK_DETECTED_VERSION STREQUAL WUJI_SDK_VERSION)
  message(FATAL_ERROR
    "Wuji SDK C ${WUJI_SDK_VERSION} required; got '${WUJI_SDK_DETECTED_VERSION}'. Set WUJI_SDK_ROOT to its extracted release.")
endif()
