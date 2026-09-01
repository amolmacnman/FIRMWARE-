# Sysbuild hook: produce a single merged.hex alongside the per-image outputs.
#
# `west flash` programs MCUboot and the application as TWO operations with a
# reset between them. On the nRF54L15 that reset lets MCUboot run and configure
# SPU protection over the application slot, so the second write to 0x10000 is
# refused with "Memory access error ... Probably a memory protection issue".
#
# Programming one merged file in one operation has no such window:
#
#   nrfutil device program --firmware <build>/merged.hex \
#       --options chip_erase_mode=ERASE_ALL --serial-number <SN>
#
# It is also the correct production artefact - a single file cannot half-apply
# and leave a board carrying a bootloader with no application.

# Sysbuild runs this file after the images are declared, so the image targets
# exist and can be depended on by name.
find_package(Python3 COMPONENTS Interpreter QUIET)

if(NOT Python3_Interpreter_FOUND)
  message(STATUS "merged.hex: no Python interpreter found - step skipped")
  return()
endif()

# The application image is named after this directory; mcuboot is fixed.
get_filename_component(_app_image_name "${CMAKE_CURRENT_LIST_DIR}" NAME)

set(_boot_hex "${CMAKE_BINARY_DIR}/mcuboot/zephyr/zephyr.hex")
set(_app_hex  "${CMAKE_BINARY_DIR}/${_app_image_name}/zephyr/zephyr.signed.hex")
set(_merged   "${CMAKE_BINARY_DIR}/merged.hex")

add_custom_target(merged_hex ALL
  COMMAND ${Python3_EXECUTABLE}
          "${CMAKE_CURRENT_LIST_DIR}/tools/merge_hex.py"
          "${_boot_hex}" "${_app_hex}" "${_merged}"
  DEPENDS ${_app_image_name} mcuboot
  COMMENT "Merging MCUboot + signed application -> merged.hex"
  VERBATIM
)
