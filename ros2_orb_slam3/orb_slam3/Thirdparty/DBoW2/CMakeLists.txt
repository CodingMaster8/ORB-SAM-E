cmake_minimum_required(VERSION 3.10)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

find_package(OpenCV REQUIRED)

set(HDRS_DBOW2
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/BowVector.h
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FORB.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FClass.h       
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FeatureVector.h
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/ScoringObject.h   
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/BowVector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FORB.cpp      
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/FeatureVector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Random.h
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Timestamp.h)
set(SRCS_DUTILS
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Random.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DUtils/Timestamp.cpp)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

