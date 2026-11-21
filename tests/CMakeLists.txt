mc_fetch_googletest()
include(GoogleTest)

# Treat GoogleTest headers as system headers so project warning flags do not apply to them.
foreach(_mc_gtest_target gtest gtest_main)
  if(TARGET ${_mc_gtest_target})
    get_target_property(_mc_gtest_includes ${_mc_gtest_target} INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(${_mc_gtest_target} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
                                                         "${_mc_gtest_includes}")
  endif()
endforeach()

add_library(mc_test_support INTERFACE)
target_include_directories(mc_test_support INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/support")
target_link_libraries(mc_test_support INTERFACE ManipulatorCore::ManipulatorCore GTest::gtest_main)

# One executable per test category; every GoogleTest case becomes a CTest test carrying the
# category label, so CI stages can run `ctest -L unit`, `ctest -L numerical`, and so on.
function(mc_add_test_suite name label)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE mc_test_support)
  mc_set_project_warnings(${name})
  if(MSVC)
    target_compile_options(${name} PRIVATE /bigobj)
  endif()
  gtest_discover_tests(
    ${name}
    DISCOVERY_MODE PRE_TEST
    DISCOVERY_TIMEOUT 120
    PROPERTIES LABELS ${label} TIMEOUT 600)
endfunction()

mc_add_test_suite(
  mc_unit_tests unit
  unit/test_random.cpp
  unit/test_linalg.cpp
  unit/test_so3.cpp
  unit/test_se3.cpp
  unit/test_rotation.cpp
  unit/test_interpolation.cpp
  unit/test_shapes.cpp
  unit/test_model.cpp
  unit/test_kinematics.cpp)

mc_add_test_suite(mc_numerical_tests numerical numerical/test_lie_numerics.cpp)
