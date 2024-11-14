# generated from ament_cmake_export_include_directories/cmake/ament_cmake_export_include_directories-extras.cmake.in

set(_exported_include_dirs "${slambot_firmware_DIR}/../../../include;${slambot_firmware_DIR}/../../../(;${slambot_firmware_DIR}/../../../)")

# append include directories to slambot_firmware_INCLUDE_DIRS
# warn about not existing paths
if(NOT _exported_include_dirs STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  foreach(_exported_include_dir ${_exported_include_dirs})
    if(NOT IS_DIRECTORY "${_exported_include_dir}")
      message(WARNING "Package 'slambot_firmware' exports the include directory '${_exported_include_dir}' which doesn't exist")
    endif()
    normalize_path(_exported_include_dir "${_exported_include_dir}")
    list(APPEND slambot_firmware_INCLUDE_DIRS "${_exported_include_dir}")
  endforeach()
endif()
