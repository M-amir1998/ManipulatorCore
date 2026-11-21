# Demo and metric-benchmark applications. Applications may print; library code never does.

add_library(mc_apps_common INTERFACE)
target_include_directories(mc_apps_common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mc_apps_common INTERFACE ManipulatorCore::ManipulatorCore)

# Data generators for the animations and figures (rendered by scripts/render).
add_executable(mc_demos demos/main.cpp demos/demos_kinematics.cpp demos/demos_planning.cpp demos/demos_control.cpp
                        demos/demo_hero.cpp)
target_link_libraries(mc_demos PRIVATE mc_apps_common)
mc_set_project_warnings(mc_demos)

# Metric benchmarks (success rates, errors, path quality, tracking) that produce Markdown tables.
add_executable(mc_metric_benchmarks benchmarks/main.cpp benchmarks/ik_benchmark.cpp benchmarks/planning_benchmark.cpp
                                    benchmarks/control_benchmark.cpp)
target_link_libraries(mc_metric_benchmarks PRIVATE mc_apps_common)
mc_set_project_warnings(mc_metric_benchmarks)

# Small command-line examples of the public API.
foreach(example basic_kinematics plan_and_execute)
  add_executable(mc_example_${example} examples/${example}.cpp)
  target_link_libraries(mc_example_${example} PRIVATE ManipulatorCore::ManipulatorCore)
  mc_set_project_warnings(mc_example_${example})
endforeach()

if(MC_BUILD_TESTS)
  # Demo smoke test: every demo must run and write its data.
  add_test(NAME demos_smoke COMMAND mc_demos "${CMAKE_BINARY_DIR}/demo_smoke" fk_ik_2r singularity rrt_2r)
  set_tests_properties(demos_smoke PROPERTIES LABELS "smoke" TIMEOUT 600)
  add_test(NAME example_basic_kinematics COMMAND mc_example_basic_kinematics)
  add_test(NAME example_plan_and_execute COMMAND mc_example_plan_and_execute)
  set_tests_properties(example_basic_kinematics example_plan_and_execute PROPERTIES LABELS "smoke" TIMEOUT 600)
endif()
