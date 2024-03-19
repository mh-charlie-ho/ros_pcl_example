cmake_minimum_required(VERSION 3.5)
project(pcl_radius_search)

add_compile_options(-std=c++14)

## Find the dependency
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  std_msgs
  # pcl_conversions
)

find_package(PCL 1.14 REQUIRED)
# find_package(pcl_conversions REQUIRED)

## Dependency for ros project  
catkin_package(CATKIN_DEPENDS 
  sensor_msgs 
  std_msgs 
  # pcl_conversions
)

include_directories(include)
add_definitions(${PCL_DEFINITIONS})


message(WARNING "${PCL_INCLUDE_DIRS}")
message(WARNING "${pcl_conversions_INCLUDE_DIRS}")


# add_library(pcl_reader src/pcl_read_file.cpp)
# target_link_libraries (pcl_reader ${PCL_LIBRARIES})

add_library(ellipsoid_clustering src/ellipsoid_clustering.cpp)
target_include_directories(ellipsoid_clustering PRIVATE
  ${PCL_INCLUDE_DIRS})
target_link_libraries (ellipsoid_clustering ${PCL_LIBRARIES})


add_library(radius_search_traversal_charlie src/radius_search_traversal_charlie.cpp)
target_include_directories(radius_search_traversal_charlie PRIVATE
  ${PCL_INCLUDE_DIRS})
target_link_libraries (radius_search_traversal_charlie ${PCL_LIBRARIES})

add_executable(main src/main.cpp)
add_dependencies(main radius_search_traversal_charlie ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(main 
  PRIVATE ${catkin_INCLUDE_DIRS}
  # PRIVATE ${pcl_conversions_INCLUDE_DIRS}
  )
target_link_libraries(main 
  ${catkin_LIBRARIES}
  # ${pcl_conversions_LIBRARIES}
  radius_search_traversal_charlie
  ellipsoid_clustering
  )

# add_executable(mainmain src/mainmain.cpp)
# # target_include_directories(mainmain PRIVATE ${catkin_INCLUDE_DIRS})
# target_link_libraries(mainmain 
#   ${PCL_LIBRARIES}
#   radius_search_traversal)

