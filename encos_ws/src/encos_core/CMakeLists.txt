cmake_minimum_required(VERSION 3.16)
project(encos_core VERSION 0.0.1 LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)
find_package(Threads REQUIRED)

add_library(encos_core
  src/bit_codec.cpp
  src/mock_transport.cpp
  src/motor_profile.cpp
  src/parameter_registry.cpp
  src/protocol_commands.cpp
  src/protocol_feedback.cpp
  src/safety_manager.cpp
  src/socketcan_transport.cpp
  src/transaction_manager.cpp
)
target_compile_features(encos_core PUBLIC cxx_std_17)
set_target_properties(encos_core PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(encos_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(encos_core PRIVATE Threads::Threads)
target_include_directories(encos_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(BUILD_TESTING)
  foreach(suite IN ITEMS bit_codec command_vectors feedback parameters invalid_frames timeouts safety transport)
    add_executable(test_${suite} test/test_${suite}.cpp)
    target_link_libraries(test_${suite} PRIVATE encos_core)
    target_compile_options(test_${suite} PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(test_${suite} PROPERTIES CXX_EXTENSIONS OFF)
    add_test(NAME ${suite} COMMAND test_${suite})
  endforeach()
endif()

install(TARGETS encos_core EXPORT encos_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/encos_core DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "control_loop.hpp" EXCLUDE
  PATTERN "motor_session.hpp" EXCLUDE
  PATTERN "trace.hpp" EXCLUDE
)

# Standalone CMake works without ROS. Ament exports the same library in Jazzy.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_targets(encos_coreTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies(Threads)
  ament_package()
else()
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/encos_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/encos_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/encos_core
  )
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/encos_coreConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/encos_core
  )
  install(EXPORT encos_coreTargets NAMESPACE encos_core::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/encos_core
  )
endif()
