cmake_minimum_required(VERSION 2.8)
project(grasp_candidates_generator)
set(CMAKE_BUILD_TYPE Release)

# Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Set compiler optimization flags
set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare")
 
# Add the headers from the <include> directory
include_directories(include)
 
# Generate the shared library from the sources
add_library(grasp_candidates_generator SHARED src/grasp_candidates_generator.cpp)

# Other libraries
add_library(antipodal src/antipodal.cpp)
add_library(cloud_camera src/cloud_camera.cpp)
add_library(config_file include/grasp_candidates_generator/config_file.cpp)
add_library(finger_hand src/finger_hand.cpp)
add_library(frame_estimator src/frame_estimator.cpp)
add_library(grasp_hypothesis src/grasp_hypothesis.cpp)
add_library(hand_search src/hand_search.cpp)
add_library(local_frame src/local_frame.cpp)
add_library(plot src/plot.cpp)
add_library(point_list src/point_list.cpp)

# This executable is for testing the shared library
add_executable(generate_candidates src/generate_candidates.cpp)
target_link_libraries(generate_candidates grasp_candidates_generator hand_search ${PCL_LIBRARIES})

# Linking for libraries
target_link_libraries(antipodal point_list)
target_link_libraries(cloud_camera ${PCL_LIBRARIES})
target_link_libraries(frame_estimator cloud_camera local_frame ${PCL_LIBRARIES})
target_link_libraries(grasp_hypothesis finger_hand)
target_link_libraries(hand_search antipodal cloud_camera finger_hand frame_estimator grasp_hypothesis local_frame plot 
  point_list ${PCL_LIBRARIES})
target_link_libraries(local_frame ${PCL_LIBRARIES})
target_link_libraries(local_frame ${PCL_LIBRARIES})
target_link_libraries(plot grasp_hypothesis local_frame ${PCL_LIBRARIES})
target_link_libraries(grasp_candidates_generator cloud_camera hand_search plot ${PCL_LIBRARIES})

# Set the location for library installation (/usr/lib in this case is not really necessary)
# Use "sudo make install" to apply
install(TARGETS grasp_candidates_generator DESTINATION /usr/lib)
install(DIRECTORY include/grasp_candidates_generator DESTINATION /usr/local/include)
