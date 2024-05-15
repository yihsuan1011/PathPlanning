cmake_minimum_required(VERSION 3.5)
project(PathPlanning)

# find_package(OpenCV REQUIRED)

# set(fcl_DIR "~/Calvin/src/calvin/fcl/build")
# list(APPEND CMAKE_PREFIX_PATH ${fcl_DIR})
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)

# set(path
#     "/usr/local/include/ompl-1.5"
#     "/usr/include/eigen3"
# )

# set(OMPL_LIBS
#     "/usr/local/lib/libompl.so.1.5.2"
#     "/usr/local/lib/libompl.so.16"
#     "/usr/local/lib/libompl.so"
# ) 


set(header
    include/Planner.h
    include/OctreeGen.h
    include/Kinematics.h
    include/RMP.h
    include/ActualNode.h
    include/GlobalPlanner.h
    include/LocalPlanner.h 
)
set(src
    src/RMP.cpp
    src/ActualNode.cpp
    # src/Planner.cpp
    src/GlobalPlanner.cpp
    src/LocalPlanner.cpp
    # src/testOctree.cpp
    # src/testGlobal.cpp
    src/testLocal.cpp
)
# include_directories(${OCTOMAP_INCLUDE_DIRS})
# target_link_libraries(${OCTOMAP_LIBRARIES})
# include_directories(${FCL_INCLUDE_DIRS})
# link_directories("/usr/local/lib/libfcl.so.0.7" "/usr/local/lib/libfcl.so.0.7.0" "/usr/local/lib/libfcl.so")

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${header} ${src})
# target_link_libraries(${PROJECT_NAME} Robot fcl ompl ${OCTOMAP_LIBRARIES})
target_link_libraries(${PROJECT_NAME} Robot fcl ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES})
# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${airobots_calvin_EXPORTED_TARGETS})

# add_executable(testOctree src/testOctree.cpp)
# target_link_libraries(testOctree ${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES})
# target_include_directories(testOctree PUBLIC ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# add_executable(testGlobal src/testGlobal.cpp)
# target_link_libraries(testGlobal ${PROJECT_NAME} Robot fcl ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES})
# target_include_directories(testGlobal PUBLIC ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})


add_executable(testLocal src/testLocal.cpp)
target_link_libraries(testLocal ${PROJECT_NAME} Robot fcl ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES})
target_include_directories(testLocal PUBLIC ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${FCL_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})