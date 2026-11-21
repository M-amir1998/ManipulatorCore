include(CMakePackageConfigHelpers)

get_property(_mc_module_targets GLOBAL PROPERTY MC_MODULE_TARGETS)

install(
  TARGETS mc_eigen ${_mc_module_targets} mc_all
  EXPORT ManipulatorCoreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/manipulatorcore" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(NOT MC_EIGEN_FROM_SYSTEM)
  # Bundle the fetched Eigen headers (MPL2) so the installed package is self-contained.
  install(DIRECTORY "${MC_EIGEN_INCLUDE_DIR}/Eigen"
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/manipulatorcore/third_party/eigen3)
  install(FILES "${MC_EIGEN_INCLUDE_DIR}/COPYING.MPL2"
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/manipulatorcore/third_party/eigen3)
endif()

set(MC_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/ManipulatorCore")

install(
  EXPORT ManipulatorCoreTargets
  NAMESPACE ManipulatorCore::
  DESTINATION "${MC_CMAKE_INSTALL_DIR}")

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/ManipulatorCoreConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/ManipulatorCoreConfig.cmake"
  INSTALL_DESTINATION "${MC_CMAKE_INSTALL_DIR}")

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/ManipulatorCoreConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMinorVersion)

install(FILES "${PROJECT_BINARY_DIR}/ManipulatorCoreConfig.cmake"
              "${PROJECT_BINARY_DIR}/ManipulatorCoreConfigVersion.cmake"
        DESTINATION "${MC_CMAKE_INSTALL_DIR}")

install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/manipulatorcore")
