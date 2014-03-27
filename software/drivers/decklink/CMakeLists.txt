cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME decklink)
include(cmake/pods.cmake)

set(DECKLINK_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/DeckLinkAPI)
include_directories(${DECKLINK_SDK_DIR})

add_executable(DecklinkCapture src/Capture.cpp src/Config.cpp ${DECKLINK_SDK_DIR}/DeckLinkAPIDispatch.cpp)

pods_install_executables(DecklinkCapture)

set(REQUIRED_PACKAGES
  bot2-core
  jpeg-utils
  lcm
)

pods_use_pkg_config_packages(DecklinkCapture ${REQUIRED_PACKAGES})
target_link_libraries(DecklinkCapture dl)
