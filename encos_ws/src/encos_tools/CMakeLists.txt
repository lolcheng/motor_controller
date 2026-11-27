cmake_minimum_required(VERSION 3.16)
project(encos_tools VERSION 0.0.1 LANGUAGES CXX)

find_package(encos_core CONFIG REQUIRED)
add_executable(encos_usb_can src/usb_can.cpp src/cli_commands.cpp)
target_link_libraries(encos_usb_can PRIVATE encos_core::encos_core)
target_compile_options(encos_usb_can PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS encos_usb_can RUNTIME DESTINATION lib/${PROJECT_NAME})

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME encos_usb_can_cli
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_usb_cli.py"
            "$<TARGET_FILE:encos_usb_can>")
  set_tests_properties(encos_usb_can_cli PROPERTIES TIMEOUT 20)
endif()

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_package()
endif()
