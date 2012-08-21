cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME math_parser)
include(cmake/pods.cmake)

FILE(GLOB EXPRTK_HDRS exprtk_8c980a16faea/exprtk.hpp)
pods_install_headers(${EXPRTK_HDRS} DESTINATION math_parser/exprtk)

# create an executable, and make it public
# add_executable(math_parser_test src/test_exprtk.cpp)
# add_executable(math_parser_test2 src/test_exprtk2.cpp)
# add_executable(testing_polymorphism src/testing_polymorphism.cpp)
# pods_install_executables(math_parser_test math_parser_test2 testing_polymorphism)
