CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(omvg C CXX)

find_package(OpenCV REQUIRED)
find_package(OpenMVG REQUIRED)
#find_package(Eigen3 REQUIRED)

# ==============================================================================
# Additional cmake find modules
# ==============================================================================
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules)
include(OptimizeForArchitecture)
AutodetectHostArchitecture()
OptimizeForArchitecture()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(main main.cpp synth.cpp)

target_include_directories(main PUBLIC
  "/usr/local/include/openMVG"
  "/usr/local/include/openMVG/third_party/eigen/"
  ${OPENMVG_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(main PUBLIC
  ${OpenCV_LIBS}
  openMVG_features
  openMVG_geometry
  openMVG_image
  openMVG_kvld
  openMVG_lInftyComputerVision
  openMVG_matching
  openMVG_matching_image_collection
  openMVG_multiview
  openMVG_sfm
  openMVG_system
  openMVG_stlplus
  stlplus
)

add_executable(reba robust_essential_ba.cpp)

target_include_directories(reba PUBLIC
  "/usr/local/include/openMVG"
  "/usr/local/include/openMVG/third_party/eigen/"
  ${OPENMVG_INCLUDE_DIRS}
)

target_link_libraries(reba PUBLIC
  ${OpenCV_LIBS}
  openMVG_features
  openMVG_geometry
  openMVG_image
  openMVG_kvld
  openMVG_lInftyComputerVision
  openMVG_matching
  openMVG_matching_image_collection
  openMVG_multiview
  openMVG_sfm
  openMVG_system
  openMVG_stlplus
  stlplus
)
