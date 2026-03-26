# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_holoassist_manager_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED holoassist_manager_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(holoassist_manager_FOUND FALSE)
  elseif(NOT holoassist_manager_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(holoassist_manager_FOUND FALSE)
  endif()
  return()
endif()
set(_holoassist_manager_CONFIG_INCLUDED TRUE)

# output package information
if(NOT holoassist_manager_FIND_QUIETLY)
  message(STATUS "Found holoassist_manager: 0.1.0 (${holoassist_manager_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'holoassist_manager' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${holoassist_manager_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(holoassist_manager_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${holoassist_manager_DIR}/${_extra}")
endforeach()
