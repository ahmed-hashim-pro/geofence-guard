cmake_minimum_required(VERSION 3.16)
project(geofence_guard_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(GFG_BUILD_TESTS "Build the core test suite" ON)
option(GFG_WERROR "Treat warnings as errors" OFF)
option(GFG_SANITIZE "Build with AddressSanitizer and UBSan" OFF)

set(GFG_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
if(GFG_WERROR)
  list(APPEND GFG_WARNINGS -Werror)
endif()

if(GFG_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

configure_file(include/geofence_guard/version.hpp.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/geofence_guard/version.hpp @ONLY)

add_library(geofence_guard_core
  src/geometry.cpp
  src/envelope.cpp
  src/guard.cpp)
add_library(geofence_guard::core ALIAS geofence_guard_core)
target_include_directories(geofence_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>)
target_compile_options(geofence_guard_core PRIVATE ${GFG_WARNINGS})
set_target_properties(geofence_guard_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(geofence-guard cli/main.cpp cli/render.cpp)
target_link_libraries(geofence-guard PRIVATE geofence_guard_core)
target_compile_options(geofence-guard PRIVATE ${GFG_WARNINGS})

include(GNUInstallDirs)
install(TARGETS geofence_guard_core geofence-guard
  EXPORT geofence_guard_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/geofence_guard DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/generated/geofence_guard/version.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/geofence_guard)
install(EXPORT geofence_guard_coreTargets
  NAMESPACE geofence_guard::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/geofence_guard_core)

include(CMakePackageConfigHelpers)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/geofence_guard_coreConfig.cmake
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/geofence_guard_coreTargets.cmake\")\n")
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/geofence_guard_coreConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/geofence_guard_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/geofence_guard_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/geofence_guard_core)

if(GFG_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(core_tests
    tests/test_geometry.cpp
    tests/test_envelope.cpp
    tests/test_guard.cpp
    tests/test_properties.cpp)
  target_link_libraries(core_tests PRIVATE geofence_guard_core GTest::gtest_main)
  target_compile_options(core_tests PRIVATE ${GFG_WARNINGS})
  include(GoogleTest)
  gtest_discover_tests(core_tests)

  # The CLI is tested as a program: exit codes and output are its interface.
  set(EX ${CMAKE_CURRENT_SOURCE_DIR}/../examples)
  add_test(NAME cli_validate COMMAND geofence-guard validate ${EX}/field.fence)
  add_test(NAME cli_check_clean COMMAND geofence-guard check ${EX}/field.fence ${EX}/survey.csv)
  add_test(NAME cli_check_refuses COMMAND geofence-guard check ${EX}/field.fence ${EX}/shortcut.csv)
  set_tests_properties(cli_check_refuses PROPERTIES WILL_FAIL ON)
  add_test(NAME cli_check_reports_reason COMMAND geofence-guard check ${EX}/field.fence ${EX}/shortcut.csv)
  set_tests_properties(cli_check_reports_reason PROPERTIES PASS_REGULAR_EXPRESSION "path_leaves_fence")
  add_test(NAME cli_bad_envelope COMMAND geofence-guard validate ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/typo.fence)
  set_tests_properties(cli_bad_envelope PROPERTIES WILL_FAIL ON)
  # render: the picture must carry the same decisions as `check`.
  add_test(NAME cli_render_marks_refusal COMMAND geofence-guard render ${EX}/field.fence ${EX}/shortcut.csv)
  set_tests_properties(cli_render_marks_refusal PROPERTIES
    PASS_REGULAR_EXPRESSION "2/3 legs ok.*REFUSED: path_leaves_fence")
  add_test(NAME cli_render_clean COMMAND geofence-guard render ${EX}/field.fence ${EX}/survey.csv)
  set_tests_properties(cli_render_clean PROPERTIES
    PASS_REGULAR_EXPRESSION "9/9 legs ok" FAIL_REGULAR_EXPRESSION "REFUSED")
  add_test(NAME cli_no_args COMMAND geofence-guard)
  set_tests_properties(cli_no_args PROPERTIES WILL_FAIL ON)
endif()
