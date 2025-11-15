# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_airspeed_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED airspeed_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(airspeed_FOUND FALSE)
  elseif(NOT airspeed_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(airspeed_FOUND FALSE)
  endif()
  return()
endif()
set(_airspeed_CONFIG_INCLUDED TRUE)

# output package information
if(NOT airspeed_FIND_QUIETLY)
  message(STATUS "Found airspeed: 0.0.0 (${airspeed_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'airspeed' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${airspeed_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(airspeed_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${airspeed_DIR}/${_extra}")
endforeach()
