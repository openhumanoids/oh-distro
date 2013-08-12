# sadly, i couldn't get this to work... needs more love.
# but everybody wants to do fpga bootloading over UDP anyway, since we
# have to seal up the jtag adapter port on the final hands.


add_custom_target(mobo-fpga-load
#                  COMMAND ${OPENOCD_INIT} -c "svf -tap spartan6.tap ${CMAKE_CURRENT_SOURCE_DIR}/mobo.svf " -c reset -c shutdown)
                  #COMMAND ${OPENOCD_INIT} -c "xsvf spartan6.tap ${CMAKE_CURRENT_SOURCE_DIR}/mobo.xsvf " -c reset -c shutdown)
                  COMMAND ${OPENOCD_INIT} -c "xsvf spartan6.tap ${CMAKE_CURRENT_SOURCE_DIR}/mobo.xsvf " -c reset -c shutdown)
                  #COMMAND . $ENV{ISEDIR}/settings64.sh && ${OPENOCD_INIT} -c xsvf spartan6 mobo.xsvf -c reset -c shutdown)

