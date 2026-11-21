mc_fetch_google_benchmark()

if(TARGET benchmark)
  get_target_property(_mc_bench_includes benchmark INTERFACE_INCLUDE_DIRECTORIES)
  set_target_properties(benchmark PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_mc_bench_includes}")
endif()

# Google Benchmark kernels. Build in Release; results are only meaningful with optimizations on.
add_executable(mc_benchmarks bench_main.cpp bench_geometry.cpp)
target_link_libraries(mc_benchmarks PRIVATE ManipulatorCore::ManipulatorCore benchmark::benchmark)
mc_set_project_warnings(mc_benchmarks)
