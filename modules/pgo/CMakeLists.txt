cmake_minimum_required(VERSION 3.16)
project(bievr_pgo VERSION 0.0.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build bievr_pgo as a shared library" ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# GTSAM and PCL live here and nowhere else: bievr_lio keeps its Eigen/Ceres/TBB
# dependency set, and the localization side links bievr_scancontext without this.
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
# VTK 9.1 as packaged on Ubuntu declares a link interface on MPI::MPI_C without
# find_package(MPI), so pulling PCL fails outright when MPI is absent. Nothing we
# use touches VTK's MPI paths, so an empty target satisfies the reference. (The
# ament packages sidestep this because pcl_conversions resolves PCL first.)
find_package(MPI QUIET)
if(NOT TARGET MPI::MPI_C)
  add_library(MPI::MPI_C INTERFACE IMPORTED)
endif()
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(bievr_lio REQUIRED)
find_package(bievr_scancontext REQUIRED)

add_library(${PROJECT_NAME} src/loop_closer.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:GNU>:-fconcepts>
  $<$<CONFIG:Release>:-O3>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  gtsam
  ${PCL_LIBRARIES}
  bievr_lio::bievr_lio
  bievr_scancontext::bievr_scancontext
)

# ---------------------------------------------------------------------------
# Install & export
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
