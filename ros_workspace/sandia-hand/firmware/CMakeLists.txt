cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sandia-hand-firmware)
enable_language(ASM)

# first, set up the cross-compiler and JTAG tool (openocd).
# i'm doing this in the cmake step instead of the "make" step so that 
# an invocation of "make clean" doesn't blow away the tools, which take
# a while to download and compile.

set (ARM_GCC_ROOT ${CMAKE_CURRENT_BINARY_DIR}/arm-2011.09)

if (NOT EXISTS ${ARM_GCC_ROOT}.tar.bz2)
  message("Downloading ARM cross-compiler...")
  file(DOWNLOAD https://sourcery.mentor.com/sgpp/lite/arm/portal/package9740/public/arm-none-eabi/arm-2011.09-69-arm-none-eabi-i686-pc-linux-gnu.tar.bz2 
       ${ARM_GCC_ROOT}.tar.bz2
       SHOW_PROGRESS)
  message("extracting ARM cross-compiler...")
  execute_process(COMMAND tar xjf ${ARM_GCC_ROOT}.tar.bz2)
endif()

set(OPENOCD_PATH ${CMAKE_CURRENT_BINARY_DIR}/openocd)

if (NOT EXISTS ${OPENOCD_PATH}_source)
  message("cloning openocd repository...")
  execute_process(COMMAND git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd ${OPENOCD_PATH}_source)
endif()

set(OPENOCD ${OPENOCD_PATH}/bin/openocd)
if (NOT EXISTS ${OPENOCD})  # this is so terrible
  message("building openocd...")
  execute_process(COMMAND bash -c "cd ${OPENOCD_PATH}_source && ./bootstrap && ./configure --enable-maintainer-mode --enable-ft2232_libftdi --prefix=${OPENOCD_PATH} && make install" RESULT_VARIABLE rv)
  message("build result: ${rv}")
endif()

#add_custom_command(OUTPUT ${ARM_GCC_ROOT}
#                   COMMAND tar xjf ${ARM_GCC_ROOT}.tar.bz2
#                   DEPENDS ${ARM_GCC_ROOT}.tar.bz2)
#add_custom_target(arm-gcc_extraction
#                  ALL DEPENDS ${ARM_GCC_ROOT})

# now clone openocd repo and get it up and running
#add_custom_command(OUTPUT ${OPENOCD_PATH}_source
#                   COMMAND git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd ${OPENOCD_PATH}_source)
#add_custom_target(openocd_clone 
#                  ALL DEPENDS ${OPENOCD_PATH}_source)

#add_custom_command(OUTPUT ${OPENOCD_PATH}/bin/openocd
#                   COMMAND cd ${OPENOCD_PATH}_source && ./bootstrap && ./configure --enable-maintainer-mode --enable-ft2232_libftdi --prefix=${OPENOCD_PATH} && make install
#                   DEPENDS ${OPENOCD_PATH}_source)
#add_custom_target(openocd_build
#                  ALL DEPENDS ${OPENOCD_PATH}/bin/openocd)

set(CMAKE_SYSTEM_NAME arm_baremetal) # heavy metal = awesome
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_C_COMPILER ${ARM_GCC_ROOT}/bin/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER ${ARM_GCC_ROOT}/bin/arm-none-eabi-gcc)
set(CMAKE_FIND_ROOT_PATH ${ARM_GCC_ROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER) # search only host
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)  # search only target
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)  # search only target
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_C_FLAGS "-O2 -std=c99 -g -mcpu=cortex-m3 -mthumb -Dprintf=iprintf")
set(CMAKE_ASM_FLAGS "-mcpu=cortex-m3 -mthumb -g")
set(SAM3X_APP_LINK_FLAGS "-g -mthumb -mcpu=cortex-m3 -T\"${PROJECT_SOURCE_DIR}/common_sam3x/sam3x8e_app.ld\" -Wl,-Map=memory.map -Wl,--cref --entry=reset_vector -Wl,--no-gc-sections -Wl,--warn-common -Wl,--unresolved-symbols=report-all -Wl,--no-undefined -Wl,--start-group -lc -lgcc -Wl,--end-group")

# sam3s common stuff
set(SAM3S_APP_LINK_FLAGS "-g -mthumb -mcpu=cortex-m3 -T\"${PROJECT_SOURCE_DIR}/common_sam3s/sam3s_app.ld\" -Wl,-Map=memory.map -Wl,--cref --entry=reset_vector -Wl,--no-gc-sections -Wl,--warn-common -Wl,--unresolved-symbols=report-all -Wl,--no-undefined")
set(SAM3S_OPENOCD_INIT ${OPENOCD} -f ${PROJECT_SOURCE_DIR}/openocd/olimex-arm-usb-tiny-h.cfg -f ${PROJECT_SOURCE_DIR}/openocd/sam3s.cfg -c init -c halt)

include_directories(asf-3.5.1_partial/sam/utils/cmsis/sam3x/include)
include_directories(asf-3.5.1_partial/thirdparty/CMSIS/Include)
include_directories(.)

add_definitions(-DDONT_USE_CMSIS_INIT)
add_subdirectory(common_sam3x)
add_subdirectory(common_sam3s)
add_subdirectory(mobo)
add_subdirectory(bl_sam3s)
add_subdirectory(fmcb)
add_subdirectory(f2)
add_subdirectory(f3)
add_subdirectory(palm)
add_subdirectory(psu)

#add_custom_target(load_fpga
#                  COMMAND . ${ISEDIR}/settings64.sh && impact -b ${ARM_GCC_ROOT}/bin/arm-none-eabi-gdb ${CMAKE_CURRENT_BINARY_DIR}/mobo_mcu.elf)

