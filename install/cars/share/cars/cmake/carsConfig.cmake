# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cars_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cars_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cars_FOUND FALSE)
  elseif(NOT cars_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cars_FOUND FALSE)
  endif()
  return()
endif()
set(_cars_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cars_FIND_QUIETLY)
  message(STATUS "Found cars: 0.0.0 (${cars_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cars' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cars_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cars_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cars_DIR}/${_extra}")
endforeach()
