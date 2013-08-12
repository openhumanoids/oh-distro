add_definitions(-DF_CPU=64000000) 
set(CMAKE_C_LINK_FLAGS ${SAM3X_APP_LINK_FLAGS})
add_library(common_sam3x STATIC console.c clocks.c led.c flash.c fpga.c enet.c config.c)
