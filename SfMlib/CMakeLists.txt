cmake_minimum_required(VERSION 3.5)

project(RGBRec)

# enable C++11 standard
set(CMAKE_CXX_STANDARD          11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -g -O1 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 ")
find_package(OpenCV 3.1 REQUIRED COMPONENTS core calib3d features2d imgproc highgui)
find_package(Boost      REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)
find_package(Ceres      REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
link_directories   (${Boost_LIBRARY_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

set(SFM_LIB_SOURCES Common.h                   Common.cpp
                    SfM.h                      SfM.cpp
                    FeatureUtils.h             FeatureUtils.cpp
                    StereoUtils.h         	   StereoUtils.cpp
                    BAUtil.h                   BAUtil.cpp
    )

add_library(SfMLibrary ${SFM_LIB_SOURCES} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(SfMTest sfmtest.cpp)

set(SFM_LIBS    
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
    )

target_link_libraries(SfMTest 
	SfMLibrary
	${SFM_LIBS}
	)

