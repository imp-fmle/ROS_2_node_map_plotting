# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pavlov_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pavlov_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pavlov_FOUND FALSE)
  elseif(NOT pavlov_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pavlov_FOUND FALSE)
  endif()
  return()
endif()
set(_pavlov_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pavlov_FIND_QUIETLY)
  message(STATUS "Found pavlov: 0.0.0 (${pavlov_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pavlov' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pavlov_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pavlov_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pavlov_DIR}/${_extra}")
endforeach()
