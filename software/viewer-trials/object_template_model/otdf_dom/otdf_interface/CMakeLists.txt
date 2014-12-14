#INSTALL(DIRECTORY include/otdf_interface DESTINATION include)

# make the header public
# install it to include/urdf_interface

FILE(GLOB OTDF_INTERFACE_HDRS include/otdf_interface/[^.]*.h)
pods_install_headers(${OTDF_INTERFACE_HDRS} DESTINATION otdf_interface)

