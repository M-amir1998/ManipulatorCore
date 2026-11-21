find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
if(NOT TARGET pybind11::module)
  # Prefer the pybind11 that ships with the active Python environment (pip install pybind11).
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE _mc_pybind11_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(_mc_pybind11_dir)
    list(APPEND CMAKE_PREFIX_PATH "${_mc_pybind11_dir}")
  endif()
endif()
mc_fetch_pybind11()

pybind11_add_module(manipulatorcore_python bindings.cpp)
set_target_properties(manipulatorcore_python PROPERTIES OUTPUT_NAME manipulatorcore LIBRARY_OUTPUT_DIRECTORY
                                                                                    "${PROJECT_BINARY_DIR}/python")
target_link_libraries(manipulatorcore_python PRIVATE ManipulatorCore::ManipulatorCore)
