cmake_minimum_required(VERSION 3.10)
project(fast_lio_sam_qn)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming 
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
include(FindOpenMP) #For Nano-GICP, Quatro #The best way to set proper compiler settings for using OpenMP in all platforms
if(OPENMP_FOUND) #For Nano-GICP, Quatro #The best way to set proper compiler settings for using OpenMP in all platforms
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

### get packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  message_filters
  nano_gicp
  quatro
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()
if(GTSAM_FOUND)
  message(WARNING "GTSAM_VER: ${GTSAM_VERSION}, and GTSAM DIR: ${GTSAM_INCLUDE_DIR}")
endif()

catkin_package() #this automatically adds/installs the results of build

### get packages' headers
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)


###########
## Build ##
###########
### main
set(COMMON_LIBS ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} ${OpenMP_LIBS} gtsam)
add_library(${PROJECT_NAME}_constructor src/constructor.cpp)
target_link_libraries(${PROJECT_NAME}_constructor ${COMMON_LIBS})
add_library(${PROJECT_NAME}_methods src/methods.cpp)
target_link_libraries(${PROJECT_NAME}_methods ${COMMON_LIBS})
add_library(${PROJECT_NAME}_callbacks src/callbacks.cpp)
target_link_libraries(${PROJECT_NAME}_callbacks ${COMMON_LIBS})

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${COMMON_LIBS} ${PROJECT_NAME}_constructor ${PROJECT_NAME}_callbacks ${PROJECT_NAME}_methods)