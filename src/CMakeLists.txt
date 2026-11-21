# Library modules. Each module is a static library exported as ManipulatorCore::<module>.
#
#   geometry    math kernels, SO(3)/SE(3), rotation conversions, interpolation, primitives
#   model       serial-manipulator description, inertia, DH tables, built-in robots, URDF I/O
#   kinematics  FK (chain, DH, PoE), Jacobians, singularity metrics, analytical + numerical IK
#   dynamics    RNEA, CRBA, ABA, forward dynamics, energy, integrators
#   trajectory  polynomial/trapezoidal/S-curve profiles, splines, Cartesian paths, time scaling
#   collision   primitive distances, GJK, AABB broad phase, robot collision checking
#   planning    joint-space RRT, RRT-Connect, RRT*, PRM, shortcutting, pose-goal pipeline
#   control     joint, computed-torque, task-space, impedance, admittance, operational-space control
#   simulation  deterministic fixed-step manipulator simulator with compliant contact

function(mc_add_module name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  set(target mc_${name})
  add_library(${target} STATIC ${ARG_SOURCES})
  add_library(ManipulatorCore::${name} ALIAS ${target})
  set_target_properties(
    ${target}
    PROPERTIES EXPORT_NAME ${name}
               OUTPUT_NAME manipulatorcore_${name}
               POSITION_INDEPENDENT_CODE ON)
  target_include_directories(${target} PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                                              "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
  target_compile_features(${target} PUBLIC cxx_std_20)
  target_link_libraries(${target} PUBLIC ${ARG_DEPS})
  mc_set_project_warnings(${target})
  set_property(GLOBAL APPEND PROPERTY MC_MODULE_TARGETS ${target})
endfunction()

mc_add_module(
  geometry
  SOURCES math/random.cpp
          math/linalg.cpp
          geometry/so3.cpp
          geometry/se3.cpp
          geometry/rotation.cpp
          geometry/interpolation.cpp
          geometry/shapes.cpp
  DEPS mc_eigen)

mc_add_module(
  model
  SOURCES model/joint.cpp
          model/inertia.cpp
          model/robot_model.cpp
          model/dh.cpp
          model/builtin_robots.cpp
          model/xml_lite.cpp
          model/urdf.cpp
  DEPS mc_geometry)

mc_add_module(
  kinematics
  SOURCES kinematics/forward_kinematics.cpp
          kinematics/jacobian.cpp
          kinematics/poe.cpp
          kinematics/singularity.cpp
          kinematics/differential_kinematics.cpp
          kinematics/inverse_kinematics.cpp
          kinematics/analytical_ik.cpp
          kinematics/workspace.cpp
          optimization/objectives.cpp
  DEPS mc_model)

mc_add_module(
  dynamics
  SOURCES dynamics/rigid_body_dynamics.cpp dynamics/planar_2r.cpp dynamics/integrators.cpp
  DEPS mc_kinematics)

# Umbrella target linking every module.
add_library(mc_all INTERFACE)
add_library(ManipulatorCore::ManipulatorCore ALIAS mc_all)
set_target_properties(mc_all PROPERTIES EXPORT_NAME ManipulatorCore)
get_property(_mc_modules GLOBAL PROPERTY MC_MODULE_TARGETS)
target_link_libraries(mc_all INTERFACE ${_mc_modules})
