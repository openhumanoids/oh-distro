#INSTALL(DIRECTORY include/urdf_interface DESTINATION include)

# make the header public
# install it to include/urdf_interface

FILE(GLOB URDF_INTERFACE_HDRS include/urdf_interface/[^.]*.h)
pods_install_headers(${URDF_INTERFACE_HDRS} DESTINATION urdf_interface)

