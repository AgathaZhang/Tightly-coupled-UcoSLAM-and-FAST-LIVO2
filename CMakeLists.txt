cmake_minimum_required(VERSION 2.8.3)
project(fast_livo)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE Debug)
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

add_compile_options(-g)
# # 设置编译类型为 Debug，并添加调试信息  25.07.10


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  #add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  #set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  #add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()


# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

find_package(Ceres REQUIRED)


#===================== teaser start ===============
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/src SRC_LIST_TEASER)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/pmc SRC_LIST_PMC)
AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/spectra/benchmark SRC_LIST_BENCHMARK)
#AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/TEASER-plusplus/teaser/tinyply/source SRC_LIST_TINYPLY)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/tinyply/third-party) 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/include)  
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/pmc/include)  
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/teaser/spectra/include)  
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/./src/3rdparty/TEASER-plusplus/examples/teaser_cpp_fpfh)  
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/TEASER-plusplus/teaser/tinyply/source)  

#find_package(OpenMP)/home/kilox/catkin_r3live/src/FAST-LIVO2/./src/3rdparty/
SET(teaser_cpp_ply ./src/3rdparty/TEASER-plusplus/examples/teaser_cpp_fpfh/teaser_cpp_fpfh.cc ${SRC_LIST_TEASER} ${SRC_LIST_BENCHMARK} 
#TEASER-plusplus/teaser/pmc/libpmc_test.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_clique_utils.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_cores.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_driver.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_graph.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_heu.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_lib.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_maxclique.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmc_utils.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmcx_maxclique_basic.cpp 
./src/3rdparty/TEASER-plusplus/teaser/pmc/pmcx_maxclique.cpp )
#===================== teaser end ===============

#//******************** backend  start ******************************
#set(GTSAM_ROOT "/home/kilox/software/gtsam2/build")
#set(GTSAM_DIR "/home/kilox/software/gtsam2/build")
set(GTSAM_ROOT "/home/kilox/software/gtsam/build")
set(GTSAM_DIR "/home/kilox/software/gtsam/build")
#find_package( GTSAMCMakeTools )
find_package(GTSAM REQUIRED QUIET)

#========kiss-matcher start ========
#find_package(robin REQUIRED)
#find_package(kiss_matcher REQUIRED)

#set_target_properties(kiss_matcher::kiss_matcher_core PROPERTIES
#    IMPORTED_LOCATION "/usr/local/lib/libkiss_matcher_core.a"
#)
#========kiss-matcher end ========
#add_subdirectory(./kiss_matcher)

#add_subdirectory("./thirdparty/Sophus")
# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)

add_service_files(FILES ImagePoseTime.srv)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem iostreams program_options system serialization timer thread)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 nav_msgs
 sensor_msgs
 std_msgs
)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport   
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include
)
#./kiss_matcher/core/



# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp ${teaser_cpp_ply})

# 添加对消息生成的依赖
add_dependencies(pre livox_ros_driver_generate_messages_cpp)

target_link_libraries(laser_mapping
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
	${Boost_SERIALIZATION_LIBRARY}
  ${CERES_LIBRARIES}
  gtsam
)
# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

#include(./src/3rdparty/find_dependencies.cmake)
#add_executable(run_kiss_matcher src/run_kiss_matcher.cc)
#target_include_directories(run_kiss_matcher
#    PUBLIC
#    ${PCL_INCLUDE_DIRS}
#)
#target_link_libraries(run_kiss_matcher
#    Eigen3::Eigen
#    TBB::tbb
#    kiss_matcher::kiss_matcher_core
#    robin::robin
#    ${PCL_LIBRARIES}
#)

# Link libraries to the executable
target_link_libraries(fastlivo_mapping

  PRIVATE
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  gtsam

  #========kiss-matcher start ========
  #run_kiss_matcher
  #kiss_matcher::kiss_matcher_core
  #robin::robin
  #========kiss-matcher end ========
)

add_dependencies(fastlivo_mapping ${PROJECT_NAME}_generate_messages_cpp)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()