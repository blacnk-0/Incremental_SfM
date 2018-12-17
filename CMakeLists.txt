cmake_minimum_required(VERSION 3.12)
project(Incremental_SfM)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Incremental_SfM main.cpp ComputeFeatures.h Definitions.h ComputeMatches.h ComputeTracks.h IncrementalSfM.h UnionFind.h third.h flat_pair_map.h TrackHelper.h TrackHelper.cpp IncrementalSfM.cpp ComputeFeatures.cpp ComputeTracks.cpp ComputeMatches.cpp)

target_link_libraries(Incremental_SfM ${OpenCV_LIBS})
