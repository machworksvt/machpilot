# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_accel_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED accel_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(accel_FOUND FALSE)
  elseif(NOT accel_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(accel_FOUND FALSE)
  endif()
  return()
endif()
set(_accel_CONFIG_INCLUDED TRUE)

# output package information
if(NOT accel_FIND_QUIETLY)
  message(STATUS "Found accel: 0.0.0 (${accel_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'accel' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${accel_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(accel_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${accel_DIR}/${_extra}")
endforeach()
