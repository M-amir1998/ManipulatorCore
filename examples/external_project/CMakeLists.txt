# Minimal downstream project consuming an installed ManipulatorCore.
#
#   cmake -S . -B build -DCMAKE_PREFIX_PATH=<install prefix>
#   cmake --build build
cmake_minimum_required(VERSION 3.21)
project(manipulatorcore_consumer LANGUAGES CXX)

find_package(ManipulatorCore 0.1 CONFIG REQUIRED)

add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_20)
target_link_libraries(consumer PRIVATE ManipulatorCore::planning ManipulatorCore::control)
