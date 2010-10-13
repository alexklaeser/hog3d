cmake_minimum_required (VERSION 2.6)
project (hog3d CXX)

# packages
include(FindPkgConfig)
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavformat libavutil libswscale)
find_package(OpenCV REQUIRED)

# includes
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${FFMPEG_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})

# sources
file(GLOB FFPP_SOURCES ffpp/*cpp)
file(GLOB OPENCV_SOURCES opencv/*cpp)
file(GLOB extractFeatures_SOURCES *cpp)

# libraries
set(external_LIBS ${FFMPEG_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
add_library(ffpp ${FFPP_SOURCES})
add_library(opencv ${OPENCV_SOURCES})

# executables
add_executable(extractFeatures ${extractFeatures_SOURCES})
target_link_libraries(extractFeatures ${external_LIBS} opencv ffpp)

# testing
set(SEQ_DIR ${CMAKE_SOURCE_DIR}/test_sequences)
set(POS ${SEQ_DIR}/walk-simple.pos)
set(AVI ${SEQ_DIR}/walk-simple.avi)
set(HOLLYWOOD ${SEQ_DIR}/hollywood2.features)
set(OUT ${CMAKE_BINARY_DIR}/out.features)
add_custom_target(test 
  ./extractFeatures -p ${POS} ${AVI} > ${OUT}
  COMMAND diff -q ${HOLLYWOOD} ${OUT}
  COMMENT "Running test to compare ${OUT} and ${HOLLYWOOD}")
add_dependencies(test extractFeatures)

# installation
install(TARGETS extractFeatures RUNTIME DESTINATION bin)
