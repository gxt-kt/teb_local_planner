cmake_minimum_required(VERSION 3.16)
project(teb)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(OpenCV 4.8 REQUIRED)
find_package(yaml-cpp REQUIRED)  
find_package(Ceres REQUIRED)

include_directories(
        /usr/include/eigen3
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)
include_directories(${CERES_INCLUDE_DIRS})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")  

include_directories(
  gxt
  inc
)

# add_library(lib_teb
#         src/teb_config.cpp
#         src/obstacles.cpp
#         src/visualization.cpp
#         src/optimal_planner.cpp
#         src/timed_elastic_band.cpp
# )

add_executable(teb main.cpp
        src/teb_config.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/optimal_planner.cpp
        src/homotopy_class_planner.cpp
        src/timed_elastic_band.cpp
        src/graph_search.cpp
  )

target_link_libraries(
        teb
        # lib_teb
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system -lboost_thread -lboost_graph
        ${OpenCV_LIBS}

    ${CERES_LIBRARIES})

      yaml-cpp

      g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d 
      g2o_csparse_extension g2o_solver_csparse
      cholmod g2o_solver_cholmod
)
